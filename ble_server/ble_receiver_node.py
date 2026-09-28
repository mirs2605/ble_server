#!/usr/bin/env python3
"""BLE清掃ゾーンを受信し、検証後にROS 2トピックへ配信する。"""

import asyncio
import sys
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped

from ble_server.ble_interface import IBleGattServer
from ble_server.bluez_server import RESPONSE_ACK, RESPONSE_NACK, BluezBleGattServer
from ble_server.chunk_assembler import AssemblerOverflowError, ChunkAssembler
from ble_server.converter import IPayloadConverter, JsonPolygonConverter


class BleReceiverNode(Node):
    """完成したBLEミッションをPolygonStampedとしてpublishするROS 2ノード。

    BLEチャンクの再構成とJSON変換を行い、全フレームが有効な場合にACKを返す。
    """

    def __init__(
        self,
        ble_server: IBleGattServer,
        converter: IPayloadConverter | None = None,
    ) -> None:
        """受信ノードを初期化する。

        Args:
            ble_server: BLE受信と応答を担当するGATTサーバー。
            converter: ペイロード変換器。省略時はROSパラメータから構成する。
        """
        super().__init__('ble_receiver_node')

        self._ble_server = ble_server

        self.declare_parameter('topic_name', '/cleaning_zone')
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('max_coord_limit_m', 1000.0)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        default_frame_id = self.get_parameter('default_frame_id').get_parameter_value().string_value
        max_coord = self.get_parameter('max_coord_limit_m').get_parameter_value().double_value

        self._converter = converter or JsonPolygonConverter(
            default_frame_id=default_frame_id,
            max_coord_abs_val=max_coord,
        )
        self._assembler = ChunkAssembler()

        # ROS 2 publisherはQoS depth 10で未処理メッセージを保持する。
        self._zone_pub = self.create_publisher(PolygonStamped, topic_name, 10)

        self.get_logger().info(f"🚀 BLE Receiver Node 起動: 配信トピック = '{topic_name}' (Default Frame: '{default_frame_id}')")

        self._ble_server.set_on_data_received_callback(self._on_ble_data_received)

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
        """完成フレームを変換・配信し、成功時はTrueを返す。"""
        self.get_logger().info(f'📥 フレーム受信: {len(frame)} bytes')

        current_time = self.get_clock().now().to_msg()
        polygon_msg = self._converter.convert_cleaning_zone(frame, stamp=current_time)

        if polygon_msg is not None:
            self._zone_pub.publish(polygon_msg)
            points_count = len(polygon_msg.polygon.points)
            frame_id = polygon_msg.header.frame_id
            self.get_logger().info(
                f"📢 清掃エリア配信完了: トピック='{self._zone_pub.topic_name}', 頂点数={points_count}, frame='{frame_id}'"
            )
            return True
        self.get_logger().error("❌ 受信データのパース・検証に失敗したため、トピックへの配信を中断しました。")
        return False


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


def main(args=None) -> None:
    """BLE受信ノードとBlueZサーバーを起動し、ROS 2 spinを実行する。"""
    rclpy.init(args=args)

    ble_server: IBleGattServer = BluezBleGattServer()
    converter: IPayloadConverter = JsonPolygonConverter()

    node = BleReceiverNode(ble_server=ble_server, converter=converter)

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
