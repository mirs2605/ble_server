"""
Abstract BLE GATT Server Interface
===================================
Bluetooth / BLE 通信基盤を抽象化し、特定のライブラリ (BlueZ, dbus-fast, bleak, mock等) に
依存しない共通インターフェースを定義する。
"""

from abc import ABC, abstractmethod
from typing import Callable, Awaitable, List

# 受信コールバック型: raw_bytes を受け取って処理を行う関数
RawDataCallback = Callable[[bytes], None]


class IBleGattServer(ABC):
    """BLE GATT Server のインターフェース"""

    @abstractmethod
    async def start(self) -> None:
        """BLE サーバー (アドバタイズおよび GATT サービス) を開始する"""
        pass

    @abstractmethod
    async def stop(self) -> None:
        """BLE サーバーを停止する"""
        pass

    @abstractmethod
    def set_on_data_received_callback(self, callback: RawDataCallback) -> None:
        """データ受信時のコールバックを登録する"""
        pass
