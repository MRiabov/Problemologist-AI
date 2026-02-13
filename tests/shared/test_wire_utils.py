import pytest
from shared.wire_utils import route_wire, calculate_length

def test_calculate_length():
    waypoints = [(0, 0, 0), (10, 0, 0), (10, 10, 0)]
    length = calculate_length(waypoints)
    assert length == 20.0

def test_route_wire_valid():
    waypoints = [(0, 0, 0), (10, 0, 0)]
    result = route_wire("w1", waypoints, gauge_awg=18)
    
    assert result.wire_id == "w1"
    assert result.total_length_mm == 10.0
    assert result.valid == True
    assert len(result.errors) == 0

def test_route_wire_invalid():
    waypoints = [(0, 0, 0)] # Too few points
    result = route_wire("w1", waypoints, gauge_awg=18)
    
    assert result.total_length_mm == 0.0
    assert result.valid == False
    assert any("zero or negative length" in e for e in result.errors)
