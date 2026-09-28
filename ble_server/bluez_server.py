"""BlueZ D-Busを使ってBLE GATTサービスとLEアドバタイズを提供する。"""

import asyncio
import logging
from typing import Optional, Any, Dict

from dbus_fast.aio import MessageBus
from dbus_fast.constants import BusType
from dbus_fast import Variant
from dbus_fast.service import ServiceInterface, method, dbus_property, PropertyAccess

from .ble_interface import IBleGattServer, RawDataCallback

logger = logging.getLogger(__name__)

# UUIDはBLEアプリのBleUuidsと一致させる。
DEFAULT_SERVICE_UUID      = "12345678-1234-1234-1234-123456789abc"
DEFAULT_CHAR_MISSION_UUID = "12345678-1234-1234-1234-123456789abd"
DEFAULT_CHAR_RESPONSE_UUID = "12345678-1234-1234-1234-123456789abe"
DEFAULT_DEVICE_NAME       = "MIRS-Robot"

RESPONSE_ACK = b"ACK"  # ミッションの検証とROS配信が成功したことを示す。
RESPONSE_NACK = b"NACK"  # ミッションを受理できなかったことを示す。

# BlueZへ公開するサービス、Characteristic、AdvertisementのObject Path。
BASE_APP_PATH  = "/com/mirs/app"
SERVICE_PATH   = f"{BASE_APP_PATH}/service0"
CHAR_PATH      = f"{SERVICE_PATH}/char0"
RESP_PATH      = f"{SERVICE_PATH}/char1"
ADV_PATH       = "/com/mirs/advertisement0"


class ObjectManagerInterface(ServiceInterface):
    """BlueZへ公開するGATTオブジェクト階層を返すD-Bus ObjectManager。"""

    def __init__(self, managed_objects: Dict[str, Dict[str, Dict[str, Variant]]]) -> None:
        """BlueZへ返すObject Pathとinterfaceの辞書を保持する。"""
        super().__init__("org.freedesktop.DBus.ObjectManager")
        self._objects = managed_objects

    @method()
    def GetManagedObjects(self) -> "a{oa{sa{sv}}}":  # type: ignore[override]
        """BlueZへ公開するObject Pathとinterfaceの辞書を返す。"""
        return self._objects


class GattService1Interface(ServiceInterface):
    """BLE GATTサービスのUUIDとPrimary属性を公開する。"""

    def __init__(self, service_uuid: str) -> None:
        """公開するGATTサービスのUUIDを設定する。"""
        super().__init__("org.bluez.GattService1")
        self._uuid = service_uuid

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        """GATTサービスのUUIDを返す。"""
        return self._uuid

    @dbus_property(PropertyAccess.READ)
    def Primary(self) -> "b":  # type: ignore[override]
        """サービスがprimary serviceであることを示す。"""
        return True


class GattCharacteristic1Interface(ServiceInterface):
    """ミッションデータを書き込むGATT Characteristicを提供する。"""

    def __init__(self, char_uuid: str, service_path: str, callback: Optional[RawDataCallback] = None) -> None:
        """UUID、所属サービス、書き込み通知先を設定する。"""
        super().__init__("org.bluez.GattCharacteristic1")
        self._uuid = char_uuid
        self._service_path = service_path
        self.callback = callback

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}") -> None:  # type: ignore[override]
        """書き込みバイト列を受信コールバックへ渡す。"""
        raw = bytes(value)
        logger.info(f"📨 WriteValue 受信: {len(raw)} bytes")
        if self.callback:
            try:
                self.callback(raw)
            except Exception as e:
                logger.error(f"コールバック実行中に例外が発生しました: {e}", exc_info=True)

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        """CharacteristicのUUIDを返す。"""
        return self._uuid

    @dbus_property(PropertyAccess.READ)
    def Service(self) -> "o":  # type: ignore[override]
        """所属GATTサービスのObject Pathを返す。"""
        return self._service_path

    @dbus_property(PropertyAccess.READ)
    def Flags(self) -> "as":  # type: ignore[override]
        """書き込み可能なCharacteristicであることを示す。"""
        return ["write"]


class GattResponseCharacteristic1Interface(ServiceInterface):
    """クライアントが処理結果を読むためのread専用Characteristic。"""

    def __init__(self, char_uuid: str, service_path: str) -> None:
        """応答CharacteristicのUUIDと所属サービスを設定する。"""
        super().__init__("org.bluez.GattCharacteristic1")
        self._uuid = char_uuid
        self._service_path = service_path
        self._value = RESPONSE_NACK

    def set_value(self, value: bytes) -> None:
        """クライアントへ返す応答Characteristicの値を更新する。"""
        self._value = bytes(value)

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":  # type: ignore[override]
        """現在設定されている処理結果をバイト列で返す。"""
        return list(self._value)

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        """応答CharacteristicのUUIDを返す。"""
        return self._uuid

    @dbus_property(PropertyAccess.READ)
    def Service(self) -> "o":  # type: ignore[override]
        """所属GATTサービスのObject Pathを返す。"""
        return self._service_path

    @dbus_property(PropertyAccess.READ)
    def Flags(self) -> "as":  # type: ignore[override]
        """読み取り専用Characteristicであることを示す。"""
        return ["read"]


class LeAdvertisement1Interface(ServiceInterface):
    """サービスUUIDとローカル名をアドバタイズするLE Advertisement。"""

    def __init__(self, service_uuid: str, local_name: str) -> None:
        """Advertisementへ含めるサービスUUIDとローカル名を設定する。"""
        super().__init__("org.bluez.LEAdvertisement1")
        self._service_uuid = service_uuid
        self._local_name = local_name

    @method()
    def Release(self) -> None:
        """BlueZがAdvertisementを解放したことを記録する。"""
        logger.info("BlueZ により Advertisement が解放されました。")

    @dbus_property(PropertyAccess.READ)
    def Type(self) -> "s":  # type: ignore[override]
        """広告形式としてperipheralを返す。"""
        return "peripheral"

    @dbus_property(PropertyAccess.READ)
    def ServiceUUIDs(self) -> "as":  # type: ignore[override]
        """アドバタイズ対象のサービスUUIDを返す。"""
        return [self._service_uuid]

    @dbus_property(PropertyAccess.READ)
    def LocalName(self) -> "s":  # type: ignore[override]
        """アドバタイズに含めるローカル名を返す。"""
        return self._local_name


class BluezBleGattServer(IBleGattServer):
    """BlueZ D-Bus APIを使うLinux向けBLE GATTサーバー実装。"""

    def __init__(
        self,
        service_uuid: str = DEFAULT_SERVICE_UUID,
        char_uuid: str = DEFAULT_CHAR_MISSION_UUID,
        response_uuid: str = DEFAULT_CHAR_RESPONSE_UUID,
        device_name: str = DEFAULT_DEVICE_NAME,
    ) -> None:
        """サービスUUID、Characteristic UUID、広告名を設定する。"""
        self._service_uuid = service_uuid
        self._char_uuid = char_uuid
        self._response_uuid = response_uuid
        self._device_name = device_name

        self._callback: Optional[RawDataCallback] = None
        self._bus: Optional[MessageBus] = None
        self._adapter_path: Optional[str] = None
        self._char_iface: Optional[GattCharacteristic1Interface] = None
        self._resp_iface: Optional[GattResponseCharacteristic1Interface] = None
        self._is_running = False

    @property
    def is_running(self) -> bool:
        """サービス登録と広告が完了していればTrueを返す。"""
        return self._is_running

    def set_on_data_received_callback(self, callback: Optional[RawDataCallback]) -> None:
        """GATT書き込みを通知するコールバックを登録または解除する。"""
        self._callback = callback
        if self._char_iface:
            self._char_iface.callback = callback

    def set_response(self, value: bytes) -> None:
        """応答Characteristicの値をACKまたはNACKへ更新する。"""
        if self._resp_iface:
            self._resp_iface.set_value(value)

    async def _detect_adapter_path(self, bus: MessageBus) -> str:
        """BlueZ D-Bus階層から最初に見つかったhciアダプタのPathを返す。

        Raises:
            RuntimeError: BlueZへ接続できない、またはアダプタがない場合。
        """
        try:
            intr = await bus.introspect("org.bluez", "/org/bluez")
        except Exception as e:
            raise RuntimeError(f"BlueZ D-Bus 探索失敗 (/org/bluez): {e}")

        for node in intr.nodes:
            if node.name.startswith("hci"):
                return f"/org/bluez/{node.name}"
        raise RuntimeError("Bluetooth アダプタ (hci0 等) が検出できませんでした。Bluetooth が有効か確認してください。")

    async def start(self) -> None:
        """GATTサービスをBlueZへ登録し、LEアドバタイズを開始する。"""
        if self._is_running:
            logger.warning("BLE サーバーは既に起動しています。")
            return

        logger.info(f"BlueZ GATT Server を起動します (Name: '{self._device_name}', Service: {self._service_uuid})...")
        try:
            self._bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
            self._adapter_path = await self._detect_adapter_path(self._bus)
            logger.info(f"Bluetooth アダプタ検出: {self._adapter_path}")

            managed_objects = {
                SERVICE_PATH: {
                    "org.bluez.GattService1": {
                        "UUID": Variant("s", self._service_uuid),
                        "Primary": Variant("b", True),
                    }
                },
                CHAR_PATH: {
                    "org.bluez.GattCharacteristic1": {
                        "UUID": Variant("s", self._char_uuid),
                        "Service": Variant("o", SERVICE_PATH),
                        "Flags": Variant("as", ["write"]),
                    }
                },
                RESP_PATH: {
                    "org.bluez.GattCharacteristic1": {
                        "UUID": Variant("s", self._response_uuid),
                        "Service": Variant("o", SERVICE_PATH),
                        "Flags": Variant("as", ["read"]),
                    }
                },
            }

            om_iface = ObjectManagerInterface(managed_objects)
            svc_iface = GattService1Interface(self._service_uuid)
            self._char_iface = GattCharacteristic1Interface(self._char_uuid, SERVICE_PATH, self._callback)
            self._resp_iface = GattResponseCharacteristic1Interface(self._response_uuid, SERVICE_PATH)
            adv_iface = LeAdvertisement1Interface(self._service_uuid, self._device_name)

            self._bus.export(BASE_APP_PATH, om_iface)
            self._bus.export(SERVICE_PATH, svc_iface)
            self._bus.export(CHAR_PATH, self._char_iface)
            self._bus.export(RESP_PATH, self._resp_iface)
            self._bus.export(ADV_PATH, adv_iface)

            intr = await self._bus.introspect("org.bluez", self._adapter_path)
            adapter = self._bus.get_proxy_object("org.bluez", self._adapter_path, intr)

            gatt_mgr = adapter.get_interface("org.bluez.GattManager1")
            await gatt_mgr.call_register_application(BASE_APP_PATH, {})
            logger.info("GATT Application 登録完了")

            adv_mgr = adapter.get_interface("org.bluez.LEAdvertisingManager1")
            await adv_mgr.call_register_advertisement(ADV_PATH, {})
            logger.info(f"BLE Advertisement 開始 (ローカル名: '{self._device_name}')")

            self._is_running = True
        except Exception as e:
            await self.stop()
            raise RuntimeError(f"BLE サーバー起動処理でエラーが発生しました: {e}") from e

    async def stop(self) -> None:
        """登録済み広告とGATTアプリケーションを解除し、D-Bus接続を閉じる。"""
        if not self._bus:
            return

        logger.info("BlueZ GATT Server を停止しています...")
        if self._adapter_path:
            try:
                intr = await self._bus.introspect("org.bluez", self._adapter_path)
                adapter = self._bus.get_proxy_object("org.bluez", self._adapter_path, intr)

                try:
                    adv_mgr = adapter.get_interface("org.bluez.LEAdvertisingManager1")
                    await adv_mgr.call_unregister_advertisement(ADV_PATH)
                except Exception as e:
                    logger.debug(f"Advertisement 解除スキップ/エラー: {e}")

                try:
                    gatt_mgr = adapter.get_interface("org.bluez.GattManager1")
                    await gatt_mgr.call_unregister_application(BASE_APP_PATH)
                except Exception as e:
                    logger.debug(f"GATT Application 解除スキップ/エラー: {e}")
            except Exception as e:
                logger.debug(f"D-Bus 終了処理例外: {e}")

        try:
            self._bus.disconnect()
        except Exception:
            pass

        self._bus = None
        self._adapter_path = None
        self._char_iface = None
        self._resp_iface = None
        self._is_running = False
        logger.info("BlueZ GATT Server 停止完了")
