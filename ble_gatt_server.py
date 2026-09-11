"""
BLE GATT Server (PC / Raspberry Pi 側)
======================================
BlueZ D-Bus API (dbus-fast) を使って GATT Peripheral として動作する。
スマホ（Android/Flutter）からの清掃範囲 JSON を受信してコンソールに表示する。

使い方:
    sudo python3 ble_gatt_server.py

依存:
    pip install bleak dbus-fast

注意:
    sudo が必要（BlueZのD-Bus APIへのアクセスのため）。
    Raspberry Pi に移植する際はそのまま使える。
    ROS 2ノードに移植する際は on_cleaning_zone_received() をPublisherに置き換える。
"""

import asyncio
import json
import logging
import sys

from dbus_fast.aio import MessageBus
from dbus_fast.constants import BusType
from dbus_fast import Variant
from dbus_fast.service import ServiceInterface, method, dbus_property

# ---------------------------------------------------------------------------
# ロギング設定
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# UUID 定義（Flutter側と一致させること）
# ---------------------------------------------------------------------------
SERVICE_UUID      = "12345678-1234-1234-1234-123456789abc"
CHAR_MISSION_UUID = "12345678-1234-1234-1234-123456789abd"

# アプリケーション・オブジェクトパス（BlueZに登録するD-Busパス）
APP_PATH       = "/com/mirs/app"
SERVICE_PATH   = "/com/mirs/app/service0"
CHAR_PATH      = "/com/mirs/app/service0/char0"
ADV_PATH       = "/com/mirs/advertisement0"

# ---------------------------------------------------------------------------
# 受信コールバック
# ---------------------------------------------------------------------------
def on_cleaning_zone_received(raw: bytes) -> None:
    """スマホから cleaning_zone JSON を受信したときに呼ばれる。"""
    try:
        text = raw.decode("utf-8")
        data = json.loads(text)
        print()
        logger.info("=" * 50)
        logger.info("✅ 清掃範囲を受信しました")
        logger.info("  type     : %s", data.get("type", "N/A"))
        logger.info("  frame_id : %s", data.get("frame_id", "N/A"))
        polygon = data.get("polygon", [])
        logger.info("  polygon  : %d 頂点", len(polygon))
        for i, pt in enumerate(polygon):
            logger.info("    [%d] x=%6.3f m,  y=%6.3f m", i, pt.get("x", 0), pt.get("y", 0))
        logger.info("=" * 50)
        print()
        # ここを ROS 2 Publisher に置き換えると /cleaning_zone に配信できる
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        logger.error("❌ JSON デコードエラー: %s  raw=%s", e, raw)
    except Exception as e:
        logger.error("❌ 予期しないエラー: %s", e)


# ---------------------------------------------------------------------------
# D-Bus サービスインターフェース
# ---------------------------------------------------------------------------

class ObjectManagerInterface(ServiceInterface):
    """org.freedesktop.DBus.ObjectManager — BlueZ が GATT を認識するために必要"""

    def __init__(self, managed_objects: dict) -> None:
        super().__init__("org.freedesktop.DBus.ObjectManager")
        self._objects = managed_objects

    @method()
    def GetManagedObjects(self) -> "a{oa{sa{sv}}}":  # type: ignore[override]
        return self._objects


class GattService1Interface(ServiceInterface):
    """org.bluez.GattService1"""

    def __init__(self) -> None:
        super().__init__("org.bluez.GattService1")

    @dbus_property(access="read")
    def UUID(self) -> "s":  # type: ignore[override]
        return SERVICE_UUID

    @dbus_property(access="read")
    def Primary(self) -> "b":  # type: ignore[override]
        return True


class GattCharacteristic1Interface(ServiceInterface):
    """org.bluez.GattCharacteristic1 — WriteValue を実装"""

    def __init__(self, callback) -> None:
        super().__init__("org.bluez.GattCharacteristic1")
        self._callback = callback

    @method()
    def WriteValue(self, value: "ay", options: "a{sv}") -> None:  # type: ignore[override]
        raw = bytes(value)
        logger.info("📨 WriteValue 受信: %d bytes", len(raw))
        self._callback(raw)

    @dbus_property(access="read")
    def UUID(self) -> "s":  # type: ignore[override]
        return CHAR_MISSION_UUID

    @dbus_property(access="read")
    def Service(self) -> "o":  # type: ignore[override]
        return SERVICE_PATH

    @dbus_property(access="read")
    def Flags(self) -> "as":  # type: ignore[override]
        return ["write"]


class LeAdvertisement1Interface(ServiceInterface):
    """org.bluez.LEAdvertisement1"""

    def __init__(self) -> None:
        super().__init__("org.bluez.LEAdvertisement1")

    @method()
    def Release(self) -> None:
        logger.info("Advertisement released by BlueZ")

    @dbus_property(access="read")
    def Type(self) -> "s":  # type: ignore[override]
        return "peripheral"

    @dbus_property(access="read")
    def ServiceUUIDs(self) -> "as":  # type: ignore[override]
        return [SERVICE_UUID]

    @dbus_property(access="read")
    def LocalName(self) -> "s":  # type: ignore[override]
        return "MIRS-Robot"

    @dbus_property(access="read")
    def Includes(self) -> "as":  # type: ignore[override]
        return ["tx-power"]


# ---------------------------------------------------------------------------
# アダプタパス取得
# ---------------------------------------------------------------------------
async def get_adapter_path(bus: MessageBus) -> str:
    """/org/bluez/hciX のパスを返す"""
    intr = await bus.introspect("org.bluez", "/org/bluez")
    for node in intr.nodes:
        if node.name.startswith("hci"):
            return f"/org/bluez/{node.name}"
    raise RuntimeError(
        "Bluetooth アダプタが見つかりません。\n"
        "  $ sudo systemctl start bluetooth\n"
        "  $ hciconfig  # hci0 が表示されるか確認"
    )


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
async def main() -> None:
    logger.info("BLE GATT Server を起動します...")
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()

    adapter_path = await get_adapter_path(bus)
    logger.info("Bluetooth アダプタ: %s", adapter_path)

    # 管理オブジェクト定義（GetManagedObjects で返す）
    managed_objects = {
        SERVICE_PATH: {
            "org.bluez.GattService1": {
                "UUID":    Variant("s", SERVICE_UUID),
                "Primary": Variant("b", True),
            }
        },
        CHAR_PATH: {
            "org.bluez.GattCharacteristic1": {
                "UUID":    Variant("s", CHAR_MISSION_UUID),
                "Service": Variant("o", SERVICE_PATH),
                "Flags":   Variant("as", ["write"]),
            }
        },
    }

    # D-Bus にインターフェースをエクスポート
    om_iface   = ObjectManagerInterface(managed_objects)
    svc_iface  = GattService1Interface()
    char_iface = GattCharacteristic1Interface(on_cleaning_zone_received)
    adv_iface  = LeAdvertisement1Interface()

    bus.export(APP_PATH,     om_iface)
    bus.export(SERVICE_PATH, svc_iface)
    bus.export(CHAR_PATH,    char_iface)
    bus.export(ADV_PATH,     adv_iface)

    # GattManager1 に GATT アプリケーションを登録
    intr = await bus.introspect("org.bluez", adapter_path)
    adapter = bus.get_proxy_object("org.bluez", adapter_path, intr)
    gatt_mgr = adapter.get_interface("org.bluez.GattManager1")
    await gatt_mgr.call_register_application(APP_PATH, {})
    logger.info("GATT Application 登録完了")

    # LEAdvertisingManager1 に Advertisement を登録
    adv_mgr = adapter.get_interface("org.bluez.LEAdvertisingManager1")
    await adv_mgr.call_register_advertisement(ADV_PATH, {})
    logger.info("BLE Advertisement 開始")

    print()
    logger.info("=" * 50)
    logger.info("🔵 BLE GATT Server 稼働中")
    logger.info("   デバイス名  : MIRS-Robot")
    logger.info("   Service UUID: %s", SERVICE_UUID)
    logger.info("   Char UUID   : %s", CHAR_MISSION_UUID)
    logger.info("   スマホからの接続を待っています...")
    logger.info("   終了: Ctrl+C")
    logger.info("=" * 50)
    print()

    try:
        await asyncio.get_event_loop().create_future()  # 無限待機
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        logger.info("停止中...")
        try:
            await adv_mgr.call_unregister_advertisement(ADV_PATH)
        except Exception:
            pass
        try:
            await gatt_mgr.call_unregister_application(APP_PATH)
        except Exception:
            pass
        bus.disconnect()
        logger.info("BLE Server 停止完了")


if __name__ == "__main__":
    if sys.platform != "linux":
        logger.error("このスクリプトは Linux (BlueZ) 環境専用です。")
        sys.exit(1)

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
