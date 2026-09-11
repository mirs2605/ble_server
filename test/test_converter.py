"""
Unit Tests for Payload Converter and BLE Interface
"""

import json
import pytest
from ble_server.converter import JsonPolygonConverter
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
