"""
Abstract BLE GATT Server Interface
===================================
Bluetooth / BLE 通信基盤を抽象化し、特定のライブラリ (BlueZ, dbus-fast, mock等) に
依存しない共通インターフェースおよびデータ型を定義する。
"""

from abc import ABC, abstractmethod
from typing import Callable, Optional

# 受信コールバック型: (生バイト列) -> None
RawDataCallback = Callable[[bytes], None]


class IBleGattServer(ABC):
    """BLE GATT Server の抽象基底インターフェース"""

    @abstractmethod
    async def start(self) -> None:
        """
        BLE サーバー (アドバタイズおよび GATT サービス) を開始する。
        
        Raises:
            RuntimeError: アダプタ未検出または D-Bus 接続失敗時
        """
        pass

    @abstractmethod
    async def stop(self) -> None:
        """BLE サーバー (アドバタイズおよび GATT サービス) を停止しリソースを解放する。"""
        pass

    @abstractmethod
    def set_on_data_received_callback(self, callback: Optional[RawDataCallback]) -> None:
        """
        データ受信時に呼び出されるコールバック関数を登録する。

        Args:
            callback: 生データ (bytes) を受け取る Callable、または解除用の None
        """
        pass

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """サーバーが稼働中かどうかを返す。"""
        pass

    def set_response(self, value: bytes) -> None:
        """送信結果の応答値 (b'ACK' / b'NACK') を保持する。

        アプリはミッション送信後に応答キャラクタリスティックを
        read するため、受信処理の成否をここに反映させること。
        応答を返せない実装では何もしない。
        """
