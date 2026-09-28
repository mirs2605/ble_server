"""BLEペイロードを検証し、ROS 2の清掃ゾーンメッセージへ変換する。"""

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
    """BLE受信バイト列をROS 2メッセージへ変換する契約。"""

    @abstractmethod
    def convert_cleaning_zone(self, raw_data: bytes, stamp: Optional[Time] = None) -> Optional[PolygonStamped]:
        """生バイト列を検証し、PolygonStampedへ変換する。

        Args:
            raw_data: UTF-8で符号化されたBLEペイロード。
            stamp: 出力headerへ設定するROS時刻。省略時は既定値を使う。

        Returns:
            変換成功時はPolygonStamped、構文または検証エラー時はNone。
        """
        pass


class JsonPolygonConverter(IPayloadConverter):
    """清掃ゾーンJSONを検証し、PolygonStampedへ変換する。

    入力は ``type`` に ``cleaning_zone``、任意の ``frame_id``、
    ``polygon`` にx/y座標の配列を持つ。頂点数、有限性、座標範囲、
    重複点、面積、自己交差を検証する。
    """

    MIN_POLYGON_VERTICES = 3
    # 許容する最小ポリゴン面積 [m^2]。
    MIN_AREA = 0.0001

    def __init__(self, default_frame_id: str = "map", max_coord_abs_val: float = 1000.0) -> None:
        """変換時に適用する既定フレームと座標制限を設定する。

        Args:
            default_frame_id: JSONにframe_idがない場合の座標フレーム。
            max_coord_abs_val: 各座標成分に許す絶対値の上限 [m]。
        """
        self._default_frame_id = default_frame_id
        self._max_coord_abs_val = max_coord_abs_val

    def convert_cleaning_zone(self, raw_data: bytes, stamp: Optional[Time] = None) -> Optional[PolygonStamped]:
        """JSONペイロードを検証し、無効な場合はNoneを返す。"""
        if not raw_data:
            logger.warning("空のデータを受信しました。")
            return None

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

        msg_type = data.get("type", "cleaning_zone")
        if msg_type != "cleaning_zone":
            logger.error(f"未知のメッセージタイプのため拒否します: '{msg_type}' (想定: 'cleaning_zone')")
            return None

        raw_polygon = data.get("polygon")
        if not isinstance(raw_polygon, list):
            logger.error(f"'polygon' フィールドが配列ではありません: {raw_polygon}")
            return None

        if len(raw_polygon) < self.MIN_POLYGON_VERTICES:
            logger.error(f"頂点数が不足しています (最低 {self.MIN_POLYGON_VERTICES} 頂点必要, 受信数: {len(raw_polygon)})")
            return None

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
    """平面ポリゴンの符号付き面積を頂点座標から求める [m^2]。"""
    area = 0.0
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return area / 2.0


def _has_duplicate_points(points: list) -> bool:
    """座標が完全一致する重複頂点があるか判定する。"""
    seen = set()
    for p in points:
        if p in seen:
            return True
        seen.add(p)
    return False


def _cross(o: tuple, a: tuple, b: tuple) -> float:
    """点oからa、bへのベクトルの2次元外積を返す。"""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _segments_cross(a: tuple, b: tuple, c: tuple, d: tuple) -> bool:
    """線分abとcdが真に交差するか判定する。端点接触は交差に含めない。"""
    ab_c = _cross(a, b, c)
    ab_d = _cross(a, b, d)
    cd_a = _cross(c, d, a)
    cd_b = _cross(c, d, b)
    return ((ab_c > 0 and ab_d < 0) or (ab_c < 0 and ab_d > 0)) and (
        (cd_a > 0 and cd_b < 0) or (cd_a < 0 and cd_b > 0)
    )


def _has_self_intersection(points: list) -> bool:
    """閉じたポリゴンの隣接しない辺に真性交差があるか判定する。"""
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
