# backlog（ble_server）

実機統合で検出した未完了項目。横断依存は `ws/TODO.md` §6。

最終更新: 2026-10-01

## 未完了

### BLE-1. launch下で `ble_receiver_node` が起動直後に死亡 [高]
- 現象（2026-10-01 03:33）: `production.launch.py` で `process started` の0.3秒後に
  `exit code 1` で死亡。以降GATT不在のためアプリから接続不可。
- 単体起動（`ros2 run ble_server ble_receiver_node`、launchと同一params）は正常。
  BlueZ登録エラーなし（hci0検出・`org.bluez` 応答あり）。
- 真のTracebackはlaunch実行ターミナルにのみ出る。次回再現時に採取すること。
- 暫定対処: コンテナ内で単独起動（`nohup ... ble_receiver_node`）。
  コンテナ再起動で消えるため恒久対処ではない。

### BLE-2. 閉路対応のコミット保留 [高]
- `ble_server/converter.py`・`test/test_converter.py`・`README.md` の閉路対応
  （末尾重複の除去・テスト2件追加、30件通過）は未コミット。
- 同一ファイルに未コミットのWIPリファクタ（dispatcher化）が既存のため、
  ファイル単位で分離できない。WIP所有者のコミット後にハンク分離すること。

### BLE-3. SIGINT終了時の二重 `shutdown` で exit 1 [中]
- `ble_receiver_node.py` の `main` が `ExternalShutdownException` 後に
  `rclpy.shutdown()` を呼んで `RCLError: already called` で落ちる
  （`coverage_bridge.py` の `main` も同型）。
- 対処案: `if rclpy.ok(): rclpy.shutdown()` ガード＋`destroy_node`の例外吸収。

## 互換注意

- 旧serverは閉路wireを重複扱いでNACKする。新旧混在時はserverを先に更新すること。
  `install/` 反映にはコンテナ内で `colcon build --packages-select ble_server` が必要。
