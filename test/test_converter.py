"""JSONペイロードの検証とROSメッセージへの変換契約を検証する。

Dispatcher経由の振り分け（cleaning_zone / initial_pose）と、
各コンバーターの検証ルールを対象とする。ROS導入済み環境で実行すること。
"""

import json
import math

from ble_server.converter import (
    DispatcherConverter,
    JsonInitialPoseConverter,
    JsonPolygonConverter,
    _has_self_intersection,
    _signed_area,
    build_default_dispatcher,
)


def _dispatch(payload: dict, **kwargs):
    dispatcher = build_default_dispatcher(**kwargs)
    return dispatcher.dispatch(json.dumps(payload).encode("utf-8"))


def _square_zone(**overrides):
    data = {
        "type": "cleaning_zone",
        "frame_id": "map",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 2.0, "y": 0.0},
            {"x": 2.0, "y": 1.0},
            {"x": 0.0, "y": 1.0},
        ],
    }
    data.update(overrides)
    return data


# --- dispatcher ---

def test_dispatcher_registers_both_types():
    dispatcher = build_default_dispatcher()
    assert set(dispatcher.registered_types) == {"cleaning_zone", "initial_pose"}


def test_dispatcher_rejects_unknown_type():
    assert _dispatch(_square_zone(type="other")) is None


def test_dispatcher_rejects_missing_type():
    data = _square_zone()
    del data["type"]
    dispatcher = build_default_dispatcher()
    assert dispatcher.dispatch(json.dumps(data).encode("utf-8")) is None


def test_dispatcher_rejects_empty_and_broken():
    dispatcher = build_default_dispatcher()
    assert dispatcher.dispatch(b"") is None
    assert dispatcher.dispatch(b"not-json") is None
    assert dispatcher.dispatch(json.dumps([1, 2]).encode("utf-8")) is None


# --- cleaning_zone ---

def test_converter_valid_square():
    result = _dispatch(_square_zone())
    assert result is not None
    assert result.topic == "/cleaning_zone"
    assert result.message.header.frame_id == "map"
    assert len(result.message.polygon.points) == 4
    assert result.message.polygon.points[0].x == 0.0
    assert result.message.polygon.points[1].x == 2.0


def test_converter_invalid_points_count():
    data = _square_zone(polygon=[{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}])
    assert _dispatch(data) is None


def test_converter_nan_value_rejection():
    dispatcher = build_default_dispatcher()
    payload = b'{"type": "cleaning_zone", "polygon": [{"x": NaN, "y": 1.0}, {"x": 2.0, "y": 1.0}, {"x": 2.0, "y": 2.0}]}'
    assert dispatcher.dispatch(payload) is None


def test_converter_out_of_bounds():
    data = _square_zone(polygon=[
        {"x": 0.0, "y": 0.0},
        {"x": 99999.0, "y": 0.0},
        {"x": 0.0, "y": 10.0},
    ])
    assert _dispatch(data, max_coord_abs_val=100.0) is None


def test_converter_rejects_duplicate_points():
    data = _square_zone(polygon=[
        {"x": 0.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
        {"x": 0.0, "y": 1.0},
    ])
    assert _dispatch(data) is None


def test_converter_accepts_closed_ring():
    # 閉路表現(末尾が始点と一致)は正規化され、開路4頂点として配信される。
    data = _square_zone(polygon=[
        {"x": 0.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
        {"x": 2.0, "y": 1.0},
        {"x": 0.0, "y": 1.0},
        {"x": 0.0, "y": 0.0},
    ])
    result = _dispatch(data)
    assert result is not None
    assert len(result.message.polygon.points) == 4


def test_converter_rejects_closed_ring_with_interior_duplicate():
    data = _square_zone(polygon=[
        {"x": 0.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
        {"x": 0.0, "y": 1.0},
        {"x": 0.0, "y": 0.0},
    ])
    assert _dispatch(data) is None


def test_converter_rejects_tiny_area():
    data = _square_zone(polygon=[
        {"x": 0.0, "y": 0.0},
        {"x": 0.001, "y": 0.0},
        {"x": 0.0, "y": 0.001},
    ])
    assert _dispatch(data) is None


def test_converter_rejects_self_intersection():
    data = _square_zone(polygon=[
        {"x": 0.0, "y": 0.0},
        {"x": 2.0, "y": 2.0},
        {"x": 0.0, "y": 2.0},
        {"x": 2.0, "y": 0.0},
    ])
    assert _dispatch(data) is None


def test_cleaning_zone_topic_override():
    result = _dispatch(_square_zone(), cleaning_zone_topic="/zone2")
    assert result is not None
    assert result.topic == "/zone2"


# --- initial_pose ---

def test_initial_pose_valid():
    result = _dispatch({"type": "initial_pose", "frame_id": "map",
                        "x": 1.0, "y": 2.0, "yaw": 0.0})
    assert result is not None
    assert result.topic == "/initialpose"
    msg = result.message
    assert msg.header.frame_id == "map"
    assert msg.pose.pose.position.x == 1.0
    assert msg.pose.pose.position.y == 2.0
    # yaw=0 -> 単位クォータニオン
    assert msg.pose.pose.orientation.z == 0.0
    assert msg.pose.pose.orientation.w == 1.0


def test_initial_pose_yaw_to_quaternion():
    converter = JsonInitialPoseConverter()
    result = converter.convert({"type": "initial_pose", "x": 0.0, "y": 0.0,
                                "yaw": math.pi / 2})
    assert result is not None
    q = result.message.pose.pose.orientation
    assert q.z == math.sin(math.pi / 4)
    assert q.w == math.cos(math.pi / 4)


def test_initial_pose_defaults_yaw_and_frame():
    converter = JsonInitialPoseConverter(default_frame_id="map")
    result = converter.convert({"type": "initial_pose", "x": 1.0, "y": 2.0})
    assert result is not None
    assert result.message.header.frame_id == "map"
    assert result.message.pose.pose.orientation.w == 1.0


def test_initial_pose_topic_override():
    result = _dispatch({"type": "initial_pose", "x": 1.0, "y": 1.0},
                       initial_pose_topic="/ble_initial_pose")
    assert result is not None
    assert result.topic == "/ble_initial_pose"


def test_initial_pose_rejects_bad_fields():
    converter = JsonInitialPoseConverter()
    # x欠落
    assert converter.convert({"type": "initial_pose", "y": 1.0}) is None
    # NaN / Inf
    assert converter.convert({"type": "initial_pose", "x": float("nan"), "y": 0.0}) is None
    assert converter.convert({"type": "initial_pose", "x": 0.0, "y": float("inf")}) is None
    # 範囲外
    assert converter.convert({"type": "initial_pose", "x": 99999.0, "y": 0.0}) is None
    # yaw不正
    assert converter.convert({"type": "initial_pose", "x": 0.0, "y": 0.0, "yaw": "abc"}) is None


def test_initial_pose_type_mismatch_rejected_by_dispatcher():
    # cleaning_zone用コンバーターにinitial_poseを直接渡しても扱えない想定だが、
    # dispatcher経由では正しく振り分けられることを確認する。
    dispatcher = DispatcherConverter(converters=[JsonPolygonConverter()])
    payload = json.dumps({"type": "initial_pose", "x": 1.0, "y": 1.0}).encode("utf-8")
    assert dispatcher.dispatch(payload) is None


# --- 幾何ヘルパー ---

def test_signed_area_square():
    assert abs(_signed_area([(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]) - 2.0) < 1e-9


def test_self_intersection_helpers():
    square = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    assert not _has_self_intersection(square)
    bowtie = [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)]
    assert _has_self_intersection(bowtie)
    concave = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.0, 1.0), (0.0, 2.0)]
    assert not _has_self_intersection(concave)
