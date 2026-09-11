"""
BlueZ / dbus-fast Implementation of IBleGattServer
===================================================
Linux (BlueZ) 上で dbus-fast を用いて BLE GATT Peripheral および Advertisement を提供する実装。
"""

import logging
from typing import Optional, Any
from dbus_fast.aio import MessageBus
from dbus_fast.constants import BusType
from dbus_fast import Variant
from dbus_fast.service import ServiceInterface, method, dbus_property, PropertyAccess

from .ble_interface import IBleGattServer, RawDataCallback

logger = logging.getLogger(__name__)

SERVICE_UUID      = "12345678-1234-1234-1234-123456789abc"
CHAR_MISSION_UUID = "12345678-1234-1234-1234-123456789abd"

APP_PATH     = "/com/mirs/app"
SERVICE_PATH = "/com/mirs/app/service0"
CHAR_PATH    = "/com/mirs/app/service0/char0"
ADV_PATH     = "/com/mirs/advertisement0"


class ObjectManagerInterface(ServiceInterface):
    def __init__(self, managed_objects: dict) -> None:
        super().__init__("org.freedesktop.DBus.ObjectManager")
        self._objects = managed_objects

    @method()
    def GetManagedObjects(self) -> "a{oa{sa{sv}}}":  # type: ignore[override]
        return self._objects


class GattService1Interface(ServiceInterface):
    def __init__(self) -> None:
        super().__init__("org.bluez.GattService1")

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        return SERVICE_UUID

    @dbus_property(PropertyAccess.READ)
    def Primary(self) -> "b":  # type: ignore[override]
        return True


class GattCharacteristic1Interface(ServiceInterface):
    def __init__(self, callback: Optional[RawDataCallback]) -> None:
        super().__init__("org.bluez.GattCharacteristic1")
        self.callback = callback

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}") -> None:  # type: ignore[override]
        raw = bytes(value)
        logger.info(f"📨 WriteValue 受信: {len(raw)} bytes")
        if self.callback:
            self.callback(raw)

    @dbus_property(PropertyAccess.READ)
    def UUID(self) -> "s":  # type: ignore[override]
        return CHAR_MISSION_UUID

    @dbus_property(PropertyAccess.READ)
    def Service(self) -> "o":  # type: ignore[override]
        return SERVICE_PATH

    @dbus_property(PropertyAccess.READ)
    def Flags(self) -> "as":  # type: ignore[override]
        return ["write"]


class LeAdvertisement1Interface(ServiceInterface):
    def __init__(self) -> None:
        super().__init__("org.bluez.LEAdvertisement1")

    @method()
    def Release(self) -> None:
        logger.info("Advertisement released by BlueZ")

    @dbus_property(PropertyAccess.READ)
    def Type(self) -> "s":  # type: ignore[override]
        return "peripheral"

    @dbus_property(PropertyAccess.READ)
    def ServiceUUIDs(self) -> "as":  # type: ignore[override]
        return [SERVICE_UUID]

    @dbus_property(PropertyAccess.READ)
    def LocalName(self) -> "s":  # type: ignore[override]
        return "MIRS-Robot"


class BluezBleGattServer(IBleGattServer):
    """BlueZ D-Bus を使った GATT サーバー実装"""

    def __init__(self, service_uuid: str = SERVICE_UUID, char_uuid: str = CHAR_MISSION_UUID) -> None:
        self._service_uuid = service_uuid
        self._char_uuid = char_uuid
        self._callback: Optional[RawDataCallback] = None
        self._bus: Optional[MessageBus] = None
        self._adapter_path: Optional[str] = None
        self._char_iface: Optional[GattCharacteristic1Interface] = None

    def set_on_data_received_callback(self, callback: RawDataCallback) -> None:
        self._callback = callback
        if self._char_iface:
            self._char_iface.callback = callback

    async def _get_adapter_path(self, bus: MessageBus) -> str:
        intr = await bus.introspect("org.bluez", "/org/bluez")
        for node in intr.nodes:
            if node.name.startswith("hci"):
                return f"/org/bluez/{node.name}"
        raise RuntimeError("Bluetooth アダプタ (hciX) が見つかりません。")

    async def start(self) -> None:
        logger.info("BlueZ GATT Server を開始しています...")
        self._bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        self._adapter_path = await self._get_adapter_path(self._bus)
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
        }

        om_iface = ObjectManagerInterface(managed_objects)
        svc_iface = GattService1Interface()
        self._char_iface = GattCharacteristic1Interface(self._callback)
        adv_iface = LeAdvertisement1Interface()

        self._bus.export(APP_PATH, om_iface)
        self._bus.export(SERVICE_PATH, svc_iface)
        self._bus.export(CHAR_PATH, self._char_iface)
        self._bus.export(ADV_PATH, adv_iface)

        intr = await self._bus.introspect("org.bluez", self._adapter_path)
        adapter = self._bus.get_proxy_object("org.bluez", self._adapter_path, intr)

        gatt_mgr = adapter.get_interface("org.bluez.GattManager1")
        await gatt_mgr.call_register_application(APP_PATH, {})
        logger.info("GATT Application 登録完了")

        adv_mgr = adapter.get_interface("org.bluez.LEAdvertisingManager1")
        await adv_mgr.call_register_advertisement(ADV_PATH, {})
        logger.info("BLE Advertisement 開始 (MIRS-Robot)")

    async def stop(self) -> None:
        if not self._bus or not self._adapter_path:
            return
        logger.info("BlueZ GATT Server を停止中...")
        try:
            intr = await self._bus.introspect("org.bluez", self._adapter_path)
            adapter = self._bus.get_proxy_object("org.bluez", self._adapter_path, intr)
            adv_mgr = adapter.get_interface("org.bluez.LEAdvertisingManager1")
            await adv_mgr.call_unregister_advertisement(ADV_PATH)
        except Exception as e:
            logger.warning(f"Advertisement 解除エラー: {e}")

        try:
            gatt_mgr = adapter.get_interface("org.bluez.GattManager1")
            await gatt_mgr.call_unregister_application(APP_PATH)
        except Exception as e:
            logger.warning(f"GATT App 解除エラー: {e}")

        self._bus.disconnect()
        logger.info("BlueZ GATT Server 停止完了")
