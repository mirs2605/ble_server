"""BLEペイロードを検証し、ROS 2メッセージへ変換する。

設計方針
--------
* ``IPayloadConverter`` はtype文字列ひとつを担当する単一責任コンバーター。
  ``handled_type`` プロパティで担当を宣言し、``convert`` で変換を行う。
* ``DispatcherConverter`` が受信JSONの ``type`` を見て適切なコンバーターへ
  ディスパッチする。新しいメッセージ型は ``DispatcherConverter`` に登録する
  だけでよく、既存コンバーターは無変更で追加できる（開放/閉鎖原則）。
* ``BleReceiverNode`` は ``DispatcherConverter`` だけを知り、配信先トピック
  の選択はコンバーター側が返す ``ConvertResult`` を介して行う。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import json
import logging
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import PolygonStamped, Point32, PoseWithCovarianceStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 変換結果コンテナ
# ---------------------------------------------------------------------------

@dataclass
class ConvertResult:
    """コンバーター1回の変換結果。

    ``topic`` と ``message`` のペアを格納する。
    変換失敗時は ``None`` の代わりにこのクラスを返さず、呼び出し元が
    戻り値 ``None`` で失敗を判断する。
    """

    topic: str
    """配信先トピック名。"""
    message: Any
    """配信するROSメッセージオブジェクト。"""


# ---------------------------------------------------------------------------
# コンバーターインターフェース
# ---------------------------------------------------------------------------

class IPayloadConverter(ABC):
    """BLE受信バイト列を *ひとつの* メッセージ型へ変換する契約。

    担当する ``type`` 文字列を :attr:`handled_type` で宣言し、
    :meth:`convert` で変換を実装する。
    """

    @property
    @abstractmethod
    def handled_type(self) -> str:
        """このコンバーターが担当するJSONの ``type`` 値。"""

    @abstractmethod
    def convert(self, data: dict, stamp: Optional[Time] = None) -> Optional[ConvertResult]:
        """パース済みdictを検証・変換し、成功時は :class:`ConvertResult` を返す。

        Args:
            data: JSON デシリアライズ済み dict（``type`` キーを含む）。
            stamp: headerへ設定するROS時刻。省略時は既定値。

        Returns:
            変換成功時は :class:`ConvertResult`、検証エラー時は ``None``。
        """


# ---------------------------------------------------------------------------
# ディスパッチャー
# ---------------------------------------------------------------------------

class DispatcherConverter:
    """``type`` フィールドで適切な :class:`IPayloadConverter` へ振り分ける。

    新しいメッセージ型は :meth:`register` で登録するだけでよく、
    既存コンバーターや受信ノードを変更する必要はない。

    Args:
        converters: 起動時に登録するコンバーターのリスト。
    """

    def __init__(self, converters: Optional[List[IPayloadConverter]] = None) -> None:
        self._registry: Dict[str, IPayloadConverter] = {}
        for c in (converters or []):
            self.register(c)

    def register(self, converter: IPayloadConverter) -> None:
        """コンバーターを登録する。同一 ``handled_type`` は上書きされる。"""
        key = converter.handled_type
        if key in self._registry:
            logger.warning(f"DispatcherConverter: '{key}' を上書き登録します。")
        self._registry[key] = converter
        logger.debug(f"DispatcherConverter: '{key}' を登録しました。")

    @property
    def registered_types(self) -> List[str]:
        """登録済みtypeキーの一覧。"""
        return list(self._registry.keys())

    def dispatch(self, raw_data: bytes, stamp: Optional[Time] = None) -> Optional[ConvertResult]:
        """バイト列をデコード・パースし、担当コンバーターへ委譲する。

        Args:
            raw_data: UTF-8で符号化されたBLEペイロード。
            stamp: ROSタイムスタンプ。

        Returns:
            変換成功時は :class:`ConvertResult`、失敗時は ``None``。
        """
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

        msg_type = data.get("type", "")
        if not msg_type:
            logger.error("'type' フィールドが存在しないか空です。")
            return None

        converter = self._registry.get(msg_type)
        if converter is None:
            logger.error(
                f"未知の type='{msg_type}' です。登録済み: {self.registered_types}"
            )
            return None

        return converter.convert(data, stamp=stamp)


# ---------------------------------------------------------------------------
# 清掃ゾーン コンバーター
# ---------------------------------------------------------------------------

class JsonPolygonConverter(IPayloadConverter):
    """``cleaning_zone`` JSONを検証し、PolygonStampedへ変換する。

    入力は ``type`` に ``cleaning_zone``、任意の ``frame_id``、
    ``polygon`` にx/y座標の配列を持つ。頂点数、有限性、座標範囲、
    重複点、面積、自己交差を検証する。閉路表現（末尾が始点と一致）は
    許容し、末尾を除去して開路に正規化する。
    """

    MIN_POLYGON_VERTICES = 3
    MIN_AREA = 0.0001
    _DEFAULT_TOPIC = "/cleaning_zone"

    def __init__(
        self,
        default_frame_id: str = "map",
        max_coord_abs_val: float = 1000.0,
        topic: str = _DEFAULT_TOPIC,
    ) -> None:
        self._default_frame_id = default_frame_id
        self._max_coord_abs_val = max_coord_abs_val
        self._topic = topic

    @property
    def handled_type(self) -> str:  # type: ignore[override]
        return "cleaning_zone"

    def convert(self, data: dict, stamp: Optional[Time] = None) -> Optional[ConvertResult]:
        """dictを検証し、失敗時は ``None`` を返す。"""
        raw_polygon = data.get("polygon")
        if not isinstance(raw_polygon, list):
            logger.error(f"'polygon' フィールドが配列ではありません: {raw_polygon}")
            return None

        if len(raw_polygon) < self.MIN_POLYGON_VERTICES:
            logger.error(
                f"頂点数が不足しています (最低 {self.MIN_POLYGON_VERTICES} 頂点必要, 受信数: {len(raw_polygon)})"
            )
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
                logger.error(
                    f"頂点 [{idx}] の座標値が許容範囲 (±{self._max_coord_abs_val}m) を超過: x={x}, y={y}"
                )
                return None

            p = Point32()
            p.x = x
            p.y = y
            p.z = z
            points.append(p)

        xy = [(p.x, p.y) for p in points]
        # 閉路表現(末尾が始点と一致)は許容し、末尾を除去して正規化する。
        # 閉形化は下流のcoverage_bridgeが担うため、topic上の形式は開路に統一する。
        if len(xy) >= 2 and xy[0] == xy[-1]:
            points = points[:-1]
            xy = xy[:-1]
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
        return ConvertResult(topic=self._topic, message=msg)


# ---------------------------------------------------------------------------
# 初期姿勢 コンバーター
# ---------------------------------------------------------------------------

class JsonInitialPoseConverter(IPayloadConverter):
    """``initial_pose`` JSONを検証し、PoseWithCovarianceStampedへ変換する。

    入力フォーマット::

        {
            "type": "initial_pose",
            "frame_id": "map",   // 省略時は default_frame_id
            "x": 1.0,
            "y": 2.0,
            "yaw": 0.0           // ラジアン。省略時は 0.0
        }

    yawからクォータニオンへの変換は z 軸回転（2D平面上の向き）のみを行う。
    """

    _DEFAULT_TOPIC = "/initialpose"

    def __init__(
        self,
        default_frame_id: str = "map",
        max_coord_abs_val: float = 1000.0,
        topic: str = _DEFAULT_TOPIC,
    ) -> None:
        self._default_frame_id = default_frame_id
        self._max_coord_abs_val = max_coord_abs_val
        self._topic = topic

    @property
    def handled_type(self) -> str:
        return "initial_pose"

    def convert(self, data: dict, stamp: Optional[Time] = None) -> Optional[ConvertResult]:
        """dictを検証し、PoseWithCovarianceStampedを返す。"""
        try:
            x = float(data["x"])
            y = float(data["y"])
        except (KeyError, ValueError, TypeError) as e:
            logger.error(f"initial_pose の x/y フィールドが不正です: {e}")
            return None

        yaw = data.get("yaw", 0.0)
        try:
            yaw = float(yaw)
        except (ValueError, TypeError) as e:
            logger.error(f"initial_pose の yaw フィールドが不正です: {e}")
            return None

        for name, val in (("x", x), ("y", y), ("yaw", yaw)):
            if math.isnan(val) or math.isinf(val):
                logger.error(f"initial_pose の '{name}' に NaN/Inf が含まれています: {val}")
                return None

        if abs(x) > self._max_coord_abs_val or abs(y) > self._max_coord_abs_val:
            logger.error(
                f"initial_pose の座標が範囲外 (±{self._max_coord_abs_val}m): x={x}, y={y}"
            )
            return None

        frame_id = str(data.get("frame_id", self._default_frame_id)).strip() or self._default_frame_id

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.pose = Pose()
        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        # 共分散はゼロ初期化（Nav2のinitialpose topicはゼロ共分散を受け入れる）。
        msg.pose.covariance = [0.0] * 36

        logger.info(
            f"✅ PoseWithCovarianceStamped 生成成功 (x={x}, y={y}, yaw={yaw:.4f}, frame_id='{frame_id}')"
        )
        return ConvertResult(topic=self._topic, message=msg)


# ---------------------------------------------------------------------------
# ファクトリー
# ---------------------------------------------------------------------------

def build_default_dispatcher(
    default_frame_id: str = "map",
    max_coord_abs_val: float = 1000.0,
    cleaning_zone_topic: str = "/cleaning_zone",
    initial_pose_topic: str = "/initialpose",
) -> DispatcherConverter:
    """デフォルト設定の :class:`DispatcherConverter` を組み立てて返す。

    Args:
        default_frame_id: JSONに frame_id がない場合の座標フレーム。
        max_coord_abs_val: 座標の絶対値上限 [m]。
        cleaning_zone_topic: cleaning_zone メッセージの配信先トピック名。
        initial_pose_topic: initial_pose メッセージの配信先トピック名。
            AMCLが購読する ``/initialpose`` に合わせるのが通常の運用。

    将来新しいメッセージ型を追加する際はここに ``register`` を追記する。
    """
    dispatcher = DispatcherConverter()
    dispatcher.register(JsonPolygonConverter(
        default_frame_id=default_frame_id,
        max_coord_abs_val=max_coord_abs_val,
        topic=cleaning_zone_topic,
    ))
    dispatcher.register(JsonInitialPoseConverter(
        default_frame_id=default_frame_id,
        max_coord_abs_val=max_coord_abs_val,
        topic=initial_pose_topic,
    ))
    return dispatcher


# ---------------------------------------------------------------------------
# ポリゴン幾何ヘルパー（モジュール内公開・テストから直接利用）
# ---------------------------------------------------------------------------

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
