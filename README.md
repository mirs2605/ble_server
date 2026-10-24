# ble_server (ROS 2 Package)

スマホ（Android/Flutter）から清掃範囲・開始位置 JSON を BLE 経由で受信し、
ROS 2 トピックへ配信する ROS 2 パッケージです。

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

他パッケージのlaunchから使う場合は、このパッケージが公開する起動APIをincludeします。
トピックや時刻設定はlaunch引数で渡せます。

```bash
ros2 launch ble_server ble_receiver.launch.py \
  cleaning_zone_topic:=/cleaning_zone initial_pose_topic:=/initialpose \
  use_sim_time:=false
```

## トピック確認

```bash
ros2 topic echo /cleaning_zone
ros2 topic echo /initialpose
```

## 受信契約

| `type` | 配信先 | 型 |
|-----|------|----|
| `cleaning_zone` | `/cleaning_zone`（変更可） | `geometry_msgs/PolygonStamped`（`frame_id=map`、頂点3以上、±1000m、閉路の末尾重複は除去、その他の重複・微小面積・自己交差を拒否） |
| `initial_pose` | `/initialpose`（変更可） | `geometry_msgs/PoseWithCovarianceStamped`（`x,y[m]`、`yaw[rad]` 省略時0、`frame_id=map`、±1000m） |

```json
{"type": "initial_pose", "frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0}
```

`/initialpose` は AMCL が直接購読するため、`coverage_bridge` を経由しません。

## UUID

| 項目 | UUID |
|-----|------|
| Service | `12345678-1234-1234-1234-123456789abc` |
| Mission Characteristic | `12345678-1234-1234-1234-123456789abd` |
