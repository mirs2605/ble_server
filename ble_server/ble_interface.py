"""BLE GATTサーバー実装が満たす共通インターフェースを定義する。"""

from abc import ABC, abstractmethod
from typing import Callable, Optional

RawDataCallback = Callable[[bytes], None]


class IBleGattServer(ABC):
    """BLE GATTサーバーの開始・停止・受信通知・応答値の契約。"""

    @abstractmethod
    async def start(self) -> None:
        """BLEアドバタイズとGATTサービスを開始する。

        Raises:
            RuntimeError: アダプタ未検出、またはD-Bus接続に失敗した場合。
        """
        pass

    @abstractmethod
    async def stop(self) -> None:
        """BLEアドバタイズとGATTサービスを停止し、通信リソースを解放する。"""
        pass

    @abstractmethod
    def set_on_data_received_callback(self, callback: Optional[RawDataCallback]) -> None:
        """受信ペイロードを通知するコールバックを登録または解除する。

        Args:
            callback: 生バイト列を受け取る関数。Noneを渡すと登録を解除する。
        """
        pass

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """GATTサービスとアドバタイズが稼働中ならTrueを返す。"""
        pass

    def set_response(self, value: bytes) -> None:
        """クライアントがreadする処理結果を設定する。

        Args:
            value: 成功を示す ``b'ACK'`` または失敗を示す ``b'NACK'``。

        応答Characteristicを持たない実装では何もしない。
        """
