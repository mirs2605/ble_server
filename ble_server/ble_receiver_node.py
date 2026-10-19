#!/usr/bin/env python3
"""BLEメッセージを受信し、型に応じたROS 2トピックへ配信する。

設計方針
--------
* ノードはメッセージの解釈を一切知らない。変換・トピック名の決定は
  :class:`~ble_server.converter.DispatcherConverter` に委ねる。
* 新しいメッセージ型を追加するには ``converter.py`` にコンバーターを
  追加して ``build_default_dispatcher`` に登録するだけでよく、
  このノードは変更不要。
"""

import asyncio
import sys
import threading
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from ble_server.ble_interface import IBleGattServer
from ble_server.bluez_server import RESPONSE_ACK, RESPONSE_NACK, BluezBleGattServer
from ble_server.chunk_assembler import AssemblerOverflowError, ChunkAssembler
from ble_server.converter import DispatcherConverter, build_default_dispatcher


class BleReceiverNode(Node):
    """BLEフレームを受信し、 :class:`DispatcherConverter` 経由でpublishするROS 2ノード。

    配信先トピックとメッセージ型はコンバーターが返す :class:`ConvertResult`
    から動的に決定する。未知のトピックが現れた際は自動でpublisherを生成する。
    """

    def __init__(
        self,
        ble_server: IBleGattServer,
        dispatcher: Optional[DispatcherConverter] = None,
    ) -> None:
        """受信ノードを初期化する。

        Args:
            ble_server: BLE受信と応答を担当するGATTサーバー。
            dispatcher: ペイロードをディスパッチするコンバーター。
                省略時は :func:`~ble_server.converter.build_default_dispatcher`
                をROSパラメータで構成して使う。
        """
        super().__init__('ble_receiver_node')

        self._ble_server = ble_server

        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('max_coord_limit_m', 1000.0)
        # 各メッセージ型の配信先トピック名。既定値はコンバーターの定数と合わせる。
        self.declare_parameter('cleaning_zone_topic', '/cleaning_zone')
        self.declare_parameter('initial_pose_topic', '/initialpose')

        default_frame_id = self.get_parameter('default_frame_id').get_parameter_value().string_value
        max_coord = self.get_parameter('max_coord_limit_m').get_parameter_value().double_value
        cleaning_zone_topic = self.get_parameter('cleaning_zone_topic').get_parameter_value().string_value
        initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self._dispatcher = dispatcher or build_default_dispatcher(
            default_frame_id=default_frame_id,
            max_coord_abs_val=max_coord,
            cleaning_zone_topic=cleaning_zone_topic,
            initial_pose_topic=initial_pose_topic,
        )
        self._assembler = ChunkAssembler()

        # トピック名 → Publisher のキャッシュ。
        # ConvertResult が返すトピックが増えるたびに自動で追加する。
        # 注意: rclpy.Node が内部で self._publishers (list) を使うため、
        #       名前が衝突しないよう _pub_cache と命名する。
        self._pub_cache: Dict[str, Publisher] = {}

        self.get_logger().info(
            f"🚀 BLE Receiver Node 起動 "
            f"(Default Frame: '{default_frame_id}', "
            f"登録済みtype: {self._dispatcher.registered_types}, "
            f"cleaning_zone→'{cleaning_zone_topic}', "
            f"initial_pose→'{initial_pose_topic}')"
        )

        self._ble_server.set_on_data_received_callback(self._on_ble_data_received)

    # ------------------------------------------------------------------
    # BLE受信コールバック
    # ------------------------------------------------------------------

    def _on_ble_data_received(self, raw_bytes: bytes) -> None:
        """WriteValueチャンクを再構成し、完成フレームの結果を応答値へ反映する。"""
        try:
            frames = self._assembler.feed(raw_bytes)
        except AssemblerOverflowError as e:
            self.get_logger().error(f'受信バッファ溢れのため破棄します: {e}')
            self._ble_server.set_response(RESPONSE_NACK)
            return
        if not frames:
            return
        ok = True
        for frame in frames:
            if not self._handle_frame(frame):
                ok = False
        self._ble_server.set_response(RESPONSE_ACK if ok else RESPONSE_NACK)

    def _handle_frame(self, frame: bytes) -> bool:
        """完成フレームをディスパッチ・配信し、成功時はTrueを返す。"""
        self.get_logger().info(f'📥 フレーム受信: {len(frame)} bytes')

        current_time = self.get_clock().now().to_msg()
        result = self._dispatcher.dispatch(frame, stamp=current_time)

        if result is None:
            self.get_logger().error(
                "❌ 受信データのパース・検証に失敗したため、トピックへの配信を中断しました。"
            )
            return False

        pub = self._get_or_create_publisher(result.topic, type(result.message))
        pub.publish(result.message)
        self.get_logger().info(
            f"📢 配信完了: topic='{result.topic}', type='{type(result.message).__name__}'"
        )
        return True

    # ------------------------------------------------------------------
    # Publisher キャッシュ
    # ------------------------------------------------------------------

    def _get_or_create_publisher(self, topic: str, msg_type) -> Publisher:
        """トピック名に対応するpublisherを返す。未登録なら生成してキャッシュする。"""
        if topic not in self._pub_cache:
            self._pub_cache[topic] = self.create_publisher(msg_type, topic, 10)
            self.get_logger().info(f"📌 Publisher を生成しました: '{topic}' ({msg_type.__name__})")
        return self._pub_cache[topic]


# ---------------------------------------------------------------------------
# BLEサーバースレッド管理（変更なし）
# ---------------------------------------------------------------------------

class BleServerManager:
    """BLEサーバー用asyncioループを専用スレッドで開始・停止する。"""

    def __init__(self, ble_server: IBleGattServer, logger=None) -> None:
        """BLEサーバーと任意のROS loggerをイベントループ管理へ結び付ける。"""
        self._server = ble_server
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._logger = logger

    def start(self) -> None:
        """イベントループ用スレッドを開始する。"""
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop,
            args=(self._loop,),
            daemon=True,
            name="BleServerThread"
        )
        self._thread.start()

    def _run_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """BLEサーバーを起動し、停止要求までイベントを処理する。"""
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._server.start())
            loop.run_forever()
        except Exception as e:
            msg = f"BLE イベントループで例外が発生しました: {e}"
            if self._logger:
                self._logger.error(msg)
            else:
                print(f"[ERROR] {msg}", file=sys.stderr)
        finally:
            try:
                loop.run_until_complete(self._server.stop())
            except Exception:
                pass
            loop.close()

    def stop(self) -> None:
        """イベントループへ停止を通知し、スレッド終了を最大3.0s待つ。"""
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)


# ---------------------------------------------------------------------------
# エントリーポイント
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """BLE受信ノードとBlueZサーバーを起動し、ROS 2 spinを実行する。"""
    rclpy.init(args=args)

    ble_server: IBleGattServer = BluezBleGattServer()

    node = BleReceiverNode(ble_server=ble_server)

    manager = BleServerManager(ble_server=ble_server, logger=node.get_logger())
    manager.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("シャットダウン要求を受信しました。")
    finally:
        node.get_logger().info("ノードおよび BLE サーバーを終了中...")
        manager.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
