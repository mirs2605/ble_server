"""
Unit Tests for Payload Converter and BLE Interface
"""

import json
import pytest
from ble_server.converter import (
    JsonPolygonConverter,
    _has_self_intersection,
    _signed_area,
)
from geometry_msgs.msg import PolygonStamped


def test_converter_valid_square():
    converter = JsonPolygonConverter(default_frame_id="map")
    payload = json.dumps({
        "type": "cleaning_zone",
        "frame_id": "map",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 2.0, "y": 0.0},
            {"x": 2.0, "y": 3.0},
            {"x": 0.0, "y": 3.0}
        ]
    }).encode("utf-8")

    result = converter.convert_cleaning_zone(payload)
    assert result is not None
    assert isinstance(result, PolygonStamped)
    assert result.header.frame_id == "map"
    assert len(result.polygon.points) == 4
    assert result.polygon.points[0].x == 0.0
    assert result.polygon.points[1].x == 2.0


def test_converter_invalid_points_count():
    converter = JsonPolygonConverter()
    # 頂点数が2つ（ポリゴンとして成立しない）
    payload = json.dumps({
        "type": "cleaning_zone",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 1.0, "y": 1.0}
        ]
    }).encode("utf-8")

    result = converter.convert_cleaning_zone(payload)
    assert result is None


def test_converter_nan_value_rejection():
    converter = JsonPolygonConverter()
    payload = b'{"type": "cleaning_zone", "polygon": [{"x": NaN, "y": 1.0}, {"x": 2.0, "y": 1.0}, {"x": 2.0, "y": 2.0}]}'

    result = converter.convert_cleaning_zone(payload)
    assert result is None


def test_converter_out_of_bounds():
    converter = JsonPolygonConverter(max_coord_abs_val=100.0)
    payload = json.dumps({
        "type": "cleaning_zone",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 99999.0, "y": 0.0},
            {"x": 0.0, "y": 10.0}
        ]
    }).encode("utf-8")

    result = converter.convert_cleaning_zone(payload)
    assert result is None


def _square_payload(**overrides):
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
    return json.dumps(data).encode("utf-8")


def test_converter_rejects_duplicate_points():
    converter = JsonPolygonConverter()
    payload = json.dumps({
        "type": "cleaning_zone",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 2.0, "y": 0.0},
            {"x": 2.0, "y": 0.0},
            {"x": 0.0, "y": 1.0},
        ],
    }).encode("utf-8")
    assert converter.convert_cleaning_zone(payload) is None


def test_converter_rejects_tiny_area():
    converter = JsonPolygonConverter()
    payload = json.dumps({
        "type": "cleaning_zone",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 0.001, "y": 0.0},
            {"x": 0.0, "y": 0.001},
        ],
    }).encode("utf-8")
    assert converter.convert_cleaning_zone(payload) is None


def test_converter_rejects_self_intersection():
    converter = JsonPolygonConverter()
    payload = json.dumps({
        "type": "cleaning_zone",
        "polygon": [
            {"x": 0.0, "y": 0.0},
            {"x": 2.0, "y": 2.0},
            {"x": 0.0, "y": 2.0},
            {"x": 2.0, "y": 0.0},
        ],
    }).encode("utf-8")
    assert converter.convert_cleaning_zone(payload) is None


def test_converter_rejects_unknown_type():
    converter = JsonPolygonConverter()
    assert converter.convert_cleaning_zone(_square_payload(type="other")) is None


def test_converter_missing_type_defaults_to_zone():
    converter = JsonPolygonConverter()
    data = json.loads(_square_payload())
    del data["type"]
    assert converter.convert_cleaning_zone(json.dumps(data).encode("utf-8")) is not None


def test_signed_area_square():
    assert abs(_signed_area([(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]) - 2.0) < 1e-9


def test_self_intersection_helpers():
    square = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    assert not _has_self_intersection(square)
    bowtie = [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)]
    assert _has_self_intersection(bowtie)
    concave = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.0, 1.0), (0.0, 2.0)]
    assert not _has_self_intersection(concave)
