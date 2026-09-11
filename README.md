# ble_server (ROS 2 Package)

スマホ（Android/Flutter）から清掃範囲 JSON を BLE 経由で受信し、ROS 2 トピック `/cleaning_zone` (`geometry_msgs/msg/PolygonStamped`) に配信する ROS 2 パッケージです。

## パッケージ構成

```
ble_server/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/ble_server
└── ble_server/
    ├── __init__.py
    ├── ble_interface.py       # IBleGattServer (抽象インターフェース)
    ├── bluez_server.py        # BluezBleGattServer (BlueZ 実装)
    ├── converter.py           # IPayloadConverter & JsonPolygonConverter
    └── ble_receiver_node.py   # ROS 2 Node 本体 (エントリーポイント)
```

## ビルド & 実行方法 (Jazzy コンテナ内)

```bash
# 1. ビルド
colcon build --packages-select ble_server
source install/setup.bash

# 2. 実行
ros2 run ble_server ble_receiver_node
```

## トピック確認

```bash
ros2 topic echo /cleaning_zone
```

## UUID

| 項目 | UUID |
|-----|------|
| Service | `12345678-1234-1234-1234-123456789abc` |
| Mission Characteristic | `12345678-1234-1234-1234-123456789abd` |
