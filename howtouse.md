
原因はBLEサーバー側のBluetoothアダプタが無効化されていることです。

確認結果:

•  Powered: no 
•  PowerState: off-blocked 
•  rfkill :  Soft blocked: yes 

そのため、BlueZが広告を登録できず、 Failed to register advertisement  が発生しています。

以下を実行してください。

sudo rfkill unblock bluetooth
bluetoothctl power on
bluetoothctl show

 bluetoothctl show  で次の状態になれば起動できます。

Powered: yes

その後、再実行します。

source mirs_workspace/install/setup.bash
ros2 run ble_server ble_receiver_node

毎回ブロックされる場合は、Bluetoothサービスを再起動してください。

sudo systemctl restart bluetooth
sudo rfkill unblock bluetooth
bluetoothctl power on

今回のエラーはアプリやROS 2のUUID設定ではなく、OSのBluetooth電源状態が原因です
