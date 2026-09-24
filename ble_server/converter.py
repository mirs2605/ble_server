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

    アプリ側 (CleaningZoneMission.isValid) と同等の幾何検証を行う。
    アプリを経由しない直接送信でも縮退ポリゴンが素通ししないこと。

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
    #: アプリ側 minArea と同一値。これ未満は縮退とみなす
    MIN_AREA = 0.0001

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

        # 2. メッセージ種別チェック (未知種別は処理せず拒否する)
        msg_type = data.get("type", "cleaning_zone")
        if msg_type != "cleaning_zone":
            logger.error(f"未知のメッセージタイプのため拒否します: '{msg_type}' (想定: 'cleaning_zone')")
            return None

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

        # 5. 幾何検証 (アプリ側 CleaningZoneMission.isValid と同等)。
        #    アプリを経由しない送信でも縮退ポリゴンが素通ししないようにする
        xy = [(p.x, p.y) for p in points]
        if _has_duplicate_points(xy):
            logger.error("重複頂点を含むため拒否します")
            return None
        if abs(_signed_area(xy)) < self.MIN_AREA:
            logger.error(f"面積が微小({self.MIN_AREA}未満)のため拒否します")
            return None
        if _has_self_intersection(xy):
            logger.error("自己交差を含むため拒否します")
            return None

        # 6. PolygonStamped メッセージの構築
        frame_id = str(data.get("frame_id", self._default_frame_id)).strip() or self._default_frame_id

        msg = PolygonStamped()
        msg.header = Header()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.polygon.points = points

        logger.info(f"✅ PolygonStamped 生成成功 (頂点数: {len(points)}, frame_id: '{frame_id}')")
        return msg


def _signed_area(points: list) -> float:
    """符号付き面積 (Shoelace)。"""
    area = 0.0
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return area / 2.0


def _has_duplicate_points(points: list) -> bool:
    """重複頂点の有無 (アプリ側と同様に完全一致で判定)。"""
    seen = set()
    for p in points:
        if p in seen:
            return True
        seen.add(p)
    return False


def _cross(o: tuple, a: tuple, b: tuple) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_cross(a: tuple, b: tuple, c: tuple, d: tuple) -> bool:
    """2線分の真性交差 (端点接触は交差とみなさない。アプリ側と同一判定)。"""
    ab_c = _cross(a, b, c)
    ab_d = _cross(a, b, d)
    cd_a = _cross(c, d, a)
    cd_b = _cross(c, d, b)
    return ((ab_c > 0 and ab_d < 0) or (ab_c < 0 and ab_d > 0)) and (
        (cd_a > 0 and cd_b < 0) or (cd_a < 0 and cd_b > 0)
    )


def _has_self_intersection(points: list) -> bool:
    """隣接しない辺同士の交差有無 (アプリ側と同一ロジック)。"""
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or j == (i + 1) % n:
                continue
            if i == 0 and j == n - 1:
                continue
            c, d = points[j], points[(j + 1) % n]
            if _segments_cross(a, b, c, d):
                return True
    return False
