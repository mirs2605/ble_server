"""
BlueZ / dbus-fast Implementation of IBleGattServer
===================================================
Linux (BlueZ) 上で D-Bus (dbus-fast) を用い、
GATT Peripheral および LE Advertisement を提供する具象クラス。
"""

import asyncio
import logging
from typing import Optional, Any, Dict

from dbus_fast.aio import MessageBus
from dbus_fast.constants import BusType
from dbus_fast import Variant
from dbus_fast.service import ServiceInterface, method, dbus_property, PropertyAccess

from .ble_interface import IBleGattServer, RawDataCallback

logger = logging.getLogger(__name__)

# デフォルト UUID (アプリ側 BleUuids と一致させること)
DEFAULT_SERVICE_UUID      = "12345678-1234-1234-1234-123456789abc"
DEFAULT_CHAR_MISSION_UUID = "12345678-1234-1234-1234-123456789abd"
DEFAULT_CHAR_RESPONSE_UUID = "12345678-1234-1234-1234-123456789abe"
DEFAULT_DEVICE_NAME       = "MIRS-Robot"

#: 応答値。アプリは送信後に response characteristic を read し、
#: b'ACK' 以外を拒否扱いにする
RESPONSE_ACK = b"ACK"
RESPONSE_NACK = b"NACK"

# D-Bus オブジェクトパス
BASE_APP_PATH  = "/com/mirs/app"
SERVICE_PATH   = f"{BASE_APP_PATH}/service0"
CHAR_PATH      = f"{SERVICE_PATH}/char0"
RESP_PATH      = f"{SERVICE_PATH}/char1"
ADV_PATH       = "/com/mirs/advertisement0"


class ObjectManagerInterface(ServiceInterface):
    """org.freedesktop.DBus.ObjectManager 実装 (BlueZ が階層構造を把握するために必要)"""

    def __init__(self, managed_objects: Dict[str, Dict[str, Dict[str, Variant]]]) -> None:
        super().__init__("org.freedesktop.DBus.ObjectManager")
        self._objects = managed_objects

    @method()
    def GetManagedObjects(self) -> "a{oa{sa{sv}}}":  # type: ignore[override]
        return self._objects


class GattService1Interface(ServiceInterface):
    """org.bluez.GattService1 実装"""

    def __init__(self, service_uuid: str) -> None:
        super().__init__("org.bluez.GattService1")
        self._uuid = service_uuid

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        return self._uuid

    @dbus_property(PropertyAccess.READ)
    def Primary(self) -> "b":  # type: ignore[override]
        return True


class GattCharacteristic1Interface(ServiceInterface):
    """org.bluez.GattCharacteristic1 実装 (WriteValue を処理)"""

    def __init__(self, char_uuid: str, service_path: str, callback: Optional[RawDataCallback] = None) -> None:
        super().__init__("org.bluez.GattCharacteristic1")
        self._uuid = char_uuid
        self._service_path = service_path
        self.callback = callback

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}") -> None:  # type: ignore[override]
        raw = bytes(value)
        logger.info(f"📨 WriteValue 受信: {len(raw)} bytes")
        if self.callback:
            try:
                self.callback(raw)
            except Exception as e:
                logger.error(f"コールバック実行中に例外が発生しました: {e}", exc_info=True)

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        return self._uuid

    @dbus_property(PropertyAccess.READ)
    def Service(self) -> "o":  # type: ignore[override]
        return self._service_path

    @dbus_property(PropertyAccess.READ)
    def Flags(self) -> "as":  # type: ignore[override]
        return ["write"]


class GattResponseCharacteristic1Interface(ServiceInterface):
    """応答用キャラクタリスティック (read専用)。

    アプリはミッション送信後にここをreadし、b'ACK' 以外を拒否扱いにする。
    受信処理の成否は BluezBleGattServer.set_response() で反映させる。
    """

    def __init__(self, char_uuid: str, service_path: str) -> None:
        super().__init__("org.bluez.GattCharacteristic1")
        self._uuid = char_uuid
        self._service_path = service_path
        self._value = RESPONSE_NACK

    def set_value(self, value: bytes) -> None:
        self._value = bytes(value)

    @method()
    def ReadValue(self, options: "a{sv}") -> "ay":  # type: ignore[override]
        return list(self._value)

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        return self._uuid

    @dbus_property(PropertyAccess.READ)
    def Service(self) -> "o":  # type: ignore[override]
        return self._service_path

    @dbus_property(PropertyAccess.READ)
    def Flags(self) -> "as":  # type: ignore[override]
        return ["read"]


class LeAdvertisement1Interface(ServiceInterface):
    """org.bluez.LEAdvertisement1 実装 (BLE 探索用)"""

    def __init__(self, service_uuid: str, local_name: str) -> None:
        super().__init__("org.bluez.LEAdvertisement1")
        self._service_uuid = service_uuid
        self._local_name = local_name

    @method()
    def Release(self) -> None:
        logger.info("BlueZ により Advertisement が解放されました。")

    @dbus_property(PropertyAccess.READ)
    def Type(self) -> "s":  # type: ignore[override]
        return "peripheral"

    @dbus_property(PropertyAccess.READ)
    def ServiceUUIDs(self) -> "as":  # type: ignore[override]
        return [self._service_uuid]

    @dbus_property(PropertyAccess.READ)
    def LocalName(self) -> "s":  # type: ignore[override]
        return self._local_name


class BluezBleGattServer(IBleGattServer):
    """BlueZ D-Bus API を利用した BLE GATT サーバーの具象実装クラス"""

    def __init__(
        self,
        service_uuid: str = DEFAULT_SERVICE_UUID,
        char_uuid: str = DEFAULT_CHAR_MISSION_UUID,
        response_uuid: str = DEFAULT_CHAR_RESPONSE_UUID,
        device_name: str = DEFAULT_DEVICE_NAME,
    ) -> None:
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
        return self._is_running

    def set_on_data_received_callback(self, callback: Optional[RawDataCallback]) -> None:
        self._callback = callback
        if self._char_iface:
            self._char_iface.callback = callback

    def set_response(self, value: bytes) -> None:
        """応答キャラクタリスティック値を更新する (ACK/NACK)。"""
        if self._resp_iface:
            self._resp_iface.set_value(value)

    async def _detect_adapter_path(self, bus: MessageBus) -> str:
        """/org/bluez 配下から利用可能な hciX アダプタを探索する"""
        try:
            intr = await bus.introspect("org.bluez", "/org/bluez")
        except Exception as e:
            raise RuntimeError(f"BlueZ D-Bus 探索失敗 (/org/bluez): {e}")

        for node in intr.nodes:
            if node.name.startswith("hci"):
                return f"/org/bluez/{node.name}"
        raise RuntimeError("Bluetooth アダプタ (hci0 等) が検出できませんでした。Bluetooth が有効か確認してください。")

    async def start(self) -> None:
        if self._is_running:
            logger.warning("BLE サーバーは既に起動しています。")
            return

        logger.info(f"BlueZ GATT Server を起動します (Name: '{self._device_name}', Service: {self._service_uuid})...")
        try:
            self._bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
            self._adapter_path = await self._detect_adapter_path(self._bus)
            logger.info(f"Bluetooth アダプタ検出: {self._adapter_path}")

            # Managed Objects 辞書の作成
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

            # D-Bus インターフェースのエクスポート
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

            # アダプタプロキシ経由で登録呼び出し
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
