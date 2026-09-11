#!/usr/bin/env python3
"""
BLE Cleaning Zone Receiver ROS 2 Node
======================================
スマホから BLE 経由で清掃エリア (Polygon JSON) を受信し、
/cleaning_zone (geometry_msgs/msg/PolygonStamped) トピックに配信する。

BLEサーバー機能 (IBleGattServer) とデータ変換機能 (IPayloadConverter) は
インターフェースを介して疎結合に設計されている。
"""

import asyncio
import threading
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped

from ble_server.ble_interface import IBleGattServer
from ble_server.bluez_server import BluezBleGattServer
from ble_server.converter import IPayloadConverter, JsonPolygonConverter


class BleReceiverNode(Node):
    def __init__(self, ble_server: IBleGattServer, converter: IPayloadConverter) -> None:
        super().__init__('ble_receiver_node')

        self._ble_server = ble_server
        self._converter = converter

        # パラメータ宣言
        self.declare_parameter('topic_name', '/cleaning_zone')
        self.declare_parameter('default_frame_id', 'map')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        # Publisher の作成
        self._zone_pub = self.create_publisher(PolygonStamped, topic_name, 10)
        self.get_logger().info(f"🚀 BLE Receiver Node 起動: 配信トピック = {topic_name}")

        # BLE 受信コールバックの登録
        self._ble_server.set_on_data_received_callback(self._on_ble_data_received)

    def _on_ble_data_received(self, raw_bytes: bytes) -> None:
        """BLE サーバーから生データを受信したときの処理 (変換 → Publish)"""
        self.get_logger().info(f"📥 BLEデータ受信: {len(raw_bytes)} bytes")

        current_time = self.get_clock().now().to_msg()
        polygon_msg = self._converter.convert_cleaning_zone(raw_bytes, stamp=current_time)

        if polygon_msg is not None:
            self._zone_pub.publish(polygon_msg)
            self.get_logger().info(
                f"📢 /cleaning_zone に清掃範囲を配信しました (頂点数: {len(polygon_msg.polygon.points)})"
            )
        else:
            self.get_logger().error("❌ データ変換に失敗したため、トピックへの配信をスキップしました")


async def run_async_server(ble_server: IBleGattServer):
    """BLE サーバー (asyncio) を起動・管理するコルーチン"""
    await ble_server.start()
    try:
        await asyncio.get_event_loop().create_future()
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        await ble_server.stop()


def ble_event_loop_thread(ble_server: IBleGattServer, loop: asyncio.AbstractEventLoop):
    """BLE サーバー用の非同期イベントループを実行するスレッド"""
    import traceback
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(run_async_server(ble_server))
    except Exception as e:
        print(f"BLE Event Loop Error: {e}", file=sys.stderr)
        traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)

    # 1. 具象クラスのインスタンス化 (必要に応じて差し替え可能)
    ble_server: IBleGattServer = BluezBleGattServer()
    converter: IPayloadConverter = JsonPolygonConverter()

    # 2. ROS 2 ノード生成 (依存性注入: DI)
    node = BleReceiverNode(ble_server=ble_server, converter=converter)

    # 3. BLE サーバー用の asyncio イベントループを別スレッドで開始
    ble_loop = asyncio.new_event_loop()
    ble_thread = threading.Thread(
        target=ble_event_loop_thread,
        args=(ble_server, ble_loop),
        daemon=True
    )
    ble_thread.start()

    # 4. ROS 2 イベントループを実行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ノード終了シグナルを受信しました")
    finally:
        # BLE スレッドのクリーンアップ
        ble_loop.call_soon_threadsafe(ble_loop.stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
