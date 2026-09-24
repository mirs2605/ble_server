# AGENTS.md — ble_server

BLE GATTサーバ（BlueZ）＋清掃ポリゴン変換。テストはROS環境が必要。

## ブランチ運用

- `main` / `develop` 直commit・直push禁止。`feature/*` → `develop` → `main` のPRのみ
- 1コミット1話題

## テスト・ビルド

```bash
colcon build --symlink-install --packages-select ble_server
# 単体テストはROS導入済み環境（コンテナ内）で実行すること。
# ホスト素のPythonでは geometry_msgs が無く collection error になる（既知・無視可）
```

## 注意

- 実機・BlueZ・D-Busが無い環境では `ble_receiver_node` は起動失敗する。手動テストは `enable_ble:=false`
- root権限を使う操作はしない
