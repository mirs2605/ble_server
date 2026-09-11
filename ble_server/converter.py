"""
Payload Converter Interface & Concrete Converters
==================================================
BLEで受信した raw bytes (JSON) を検証し、ROS 2 メッセージ (geometry_msgs/PolygonStamped 等) に変換する。
"""

from abc import ABC, abstractmethod
import json
import logging
from typing import Optional

from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header

logger = logging.getLogger(__name__)


class IPayloadConverter(ABC):
    """BLE Payload から ROS 2 メッセージへの変換器インターフェース"""

    @abstractmethod
    def convert_cleaning_zone(self, raw_data: bytes, stamp=None) -> Optional[PolygonStamped]:
        """
        raw bytes (JSON) を PolygonStamped に変換する。
        不正なデータの場合は None を返す。
        """
        pass


class JsonPolygonConverter(IPayloadConverter):
    """
    JSON 形式の清掃範囲データを PolygonStamped に変換する標準実装。
    
    期待するJSONフォーマット:
    {
      "type": "cleaning_zone",
      "frame_id": "map",
      "polygon": [
        {"x": 1.0, "y": 1.0},
        {"x": 4.0, "y": 1.0},
        {"x": 4.0, "y": 3.0},
        {"x": 1.0, "y": 3.0}
      ]
    }
    """

    def __init__(self, default_frame_id: str = "map") -> None:
        self._default_frame_id = default_frame_id

    def convert_cleaning_zone(self, raw_data: bytes, stamp=None) -> Optional[PolygonStamped]:
        try:
            text = raw_data.decode("utf-8")
            data = json.loads(text)
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            logger.error(f"❌ JSONパース失敗: {e}")
            return None

        # バリデーション
        msg_type = data.get("type", "cleaning_zone")
        if msg_type != "cleaning_zone":
            logger.warning(f"⚠️ 未知のメッセージタイプ: {msg_type}")

        polygon_points = data.get("polygon")
        if not isinstance(polygon_points, list) or len(polygon_points) < 3:
            logger.error(f"❌ 不正なポリゴンデータ (最低3頂点必要): {polygon_points}")
            return None

        frame_id = data.get("frame_id", self._default_frame_id)

        msg = PolygonStamped()
        msg.header = Header()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        for idx, pt in enumerate(polygon_points):
            if not isinstance(pt, dict) or "x" not in pt or "y" not in pt:
                logger.error(f"❌ 頂点フォーマットエラー [{idx}]: {pt}")
                return None
            try:
                x = float(pt["x"])
                y = float(pt["y"])
                z = float(pt.get("z", 0.0))
            except (ValueError, TypeError) as e:
                logger.error(f"❌ 座標の数値変換失敗 [{idx}]: {e}")
                return None

            p = Point32()
            p.x = x
            p.y = y
            p.z = z
            msg.polygon.points.append(p)

        logger.info(f"✅ PolygonStamped 生成成功 (頂点数: {len(msg.polygon.points)}, frame: {frame_id})")
        return msg
