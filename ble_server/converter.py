"""
Payload Converter Interface & Concrete Implementations
======================================================
BLEで受信した生バイト列 (JSON 等) の構文解析・バリデーションを行い、
ROS 2 メッセージ (geometry_msgs/PolygonStamped 等) に安全に変換する。
"""

from abc import ABC, abstractmethod
import json
import logging
import math
from typing import Optional, List, Dict, Any

from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

logger = logging.getLogger(__name__)


class IPayloadConverter(ABC):
    """BLE Payload から ROS 2 メッセージへの変換器インターフェース"""

    @abstractmethod
    def convert_cleaning_zone(self, raw_data: bytes, stamp: Optional[Time] = None) -> Optional[PolygonStamped]:
        """
        生バイト列を PolygonStamped メッセージに変換する。
        
        Args:
            raw_data: 受信したバイト列
            stamp: 付与するタイムスタンプ (省略時は空)
            
        Returns:
            正常時: geometry_msgs.msg.PolygonStamped
            パース/検証失敗時: None
        """
        pass


class JsonPolygonConverter(IPayloadConverter):
    """
    JSON 形式の清掃範囲データを検証し PolygonStamped に変換する標準実装。

    【期待フォーマット】
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

    MIN_POLYGON_VERTICES = 3

    def __init__(self, default_frame_id: str = "map", max_coord_abs_val: float = 1000.0) -> None:
        """
        Args:
            default_frame_id: JSON内に frame_id がない場合に使用するデフォルトフレーム
            max_coord_abs_val: 外れ値・異常値排除用の許容絶対座標範囲 (メートル)
        """
        self._default_frame_id = default_frame_id
        self._max_coord_abs_val = max_coord_abs_val

    def convert_cleaning_zone(self, raw_data: bytes, stamp: Optional[Time] = None) -> Optional[PolygonStamped]:
        if not raw_data:
            logger.warning("空のデータを受信しました。")
            return None

        # 1. JSON デコード
        try:
            text = raw_data.decode("utf-8")
            data: Any = json.loads(text)
        except UnicodeDecodeError as e:
            logger.error(f"UTF-8 デコード失敗: {e}")
            return None
        except json.JSONDecodeError as e:
            logger.error(f"JSON パース失敗: {e} (生データ: {raw_data[:100]!r})")
            return None

        if not isinstance(data, dict):
            logger.error(f"JSON ルートがオブジェクトではありません: {type(data)}")
            return None

        # 2. メッセージ種別チェック
        msg_type = data.get("type", "cleaning_zone")
        if msg_type != "cleaning_zone":
            logger.warning(f"未知のメッセージタイプを受信しました: '{msg_type}' (想定: 'cleaning_zone')")

        # 3. ポリゴン頂点リストの検証
        raw_polygon = data.get("polygon")
        if not isinstance(raw_polygon, list):
            logger.error(f"'polygon' フィールドが配列ではありません: {raw_polygon}")
            return None

        if len(raw_polygon) < self.MIN_POLYGON_VERTICES:
            logger.error(f"頂点数が不足しています (最低 {self.MIN_POLYGON_VERTICES} 頂点必要, 受信数: {len(raw_polygon)})")
            return None

        # 4. 各頂点のパースと座標検証 (NaN, Inf, 異常値チェック)
        points: List[Point32] = []
        for idx, pt in enumerate(raw_polygon):
            if not isinstance(pt, dict) or "x" not in pt or "y" not in pt:
                logger.error(f"頂点 [{idx}] のキーが不正です (x, y が必要): {pt}")
                return None

            try:
                x = float(pt["x"])
                y = float(pt["y"])
                z = float(pt.get("z", 0.0))
            except (ValueError, TypeError) as e:
                logger.error(f"頂点 [{idx}] の数値変換に失敗しました: {e} ({pt})")
                return None

            # 数値整合性チェック
            if any(math.isnan(v) or math.isinf(v) for v in (x, y, z)):
                logger.error(f"頂点 [{idx}] に NaN または Inf が含まれています: x={x}, y={y}, z={z}")
                return None

            if any(abs(v) > self._max_coord_abs_val for v in (x, y, z)):
                logger.error(f"頂点 [{idx}] の座標値が許容範囲 (±{self._max_coord_abs_val}m) を超過しています: x={x}, y={y}")
                return None

            p = Point32()
            p.x = x
            p.y = y
            p.z = z
            points.append(p)

        # 5. PolygonStamped メッセージの構築
        frame_id = str(data.get("frame_id", self._default_frame_id)).strip() or self._default_frame_id

        msg = PolygonStamped()
        msg.header = Header()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.polygon.points = points

        logger.info(f"✅ PolygonStamped 生成成功 (頂点数: {len(points)}, frame_id: '{frame_id}')")
        return msg
