import pytest
import numpy as np
from build123d import Box, Compound, Vector
from shared.wire_utils import (
    get_awg_properties,
    calculate_path_length,
    generate_spline_points,
    check_wire_clearance,
    route_wire,
)


def test_awg_lookup():
    """T013: Verify AWG property lookup."""
    props_22 = get_awg_properties(22)
    assert props_22["diameter_mm"] == 0.644
    assert props_22["max_current_a"] == 7.0

    # Test fallback
    props_unknown = get_awg_properties(100)  # Ridiculously thin
    assert props_unknown["diameter_mm"] < 0.1
    assert props_unknown["resistance_ohm_m"] > 10.0


def test_path_length():
    """T012: Verify length calculation."""
    waypoints = [(0, 0, 0), (10, 0, 0), (10, 10, 0)]

    # Polyline length should be exactly 20
    len_poly = calculate_path_length(waypoints, use_spline=False)
    assert pytest.approx(len_poly) == 20.0

    # Spline length should be slightly different (usually shorter for these corners)
    len_spline = calculate_path_length(waypoints, use_spline=True)
    assert len_spline != 20.0
    assert len_spline > 10.0


def test_spline_generation():
    """T010: Verify spline point sampling."""
    waypoints = [(0, 0, 0), (10, 10, 10), (20, 0, 0)]
    samples = generate_spline_points(waypoints, samples_per_mm=1.0)

    assert len(samples) > 2
    assert samples[0] == (0.0, 0.0, 0.0)
    # Floating point precision might make the last point slightly off but close
    assert pytest.approx(samples[-1][0]) == 20.0


def test_wire_clearance():
    """T011: Verify collision detection."""
    # Create a 10x10x10 box at origin
    box = Box(10, 10, 10)
    assembly = Compound(children=[box])

    # Path far away: (10, 10, 10) is a corner, so (20, 20, 20) to (30, 20, 20) is clear
    clear_path = [(20, 20, 20), (30, 20, 20)]
    assert check_wire_clearance(clear_path, assembly, clearance_mm=2.0) is True

    # Path passing through the box
    intersect_path = [(-10, 0, 0), (10, 0, 0)]
    assert check_wire_clearance(intersect_path, assembly, clearance_mm=1.0) is False

    # Path very close but not intersecting (box is -5 to 5)
    # Point at (0, 6, 0) is 1mm away from face at y=5
    close_path = [(-10, 6, 0), (10, 6, 0)]
    assert check_wire_clearance(close_path, assembly, clearance_mm=0.5) is True
    assert check_wire_clearance(close_path, assembly, clearance_mm=2.0) is False


def test_wire_clearance_endpoints():
    """T011: Verify that intersections near endpoints are ignored."""
    box = Box(10, 10, 10)
    assembly = Compound(children=[box])

    # Start INSIDE the box, but exit immediately and stay clear
    # (0,0,0) is center.
    # Path from center to far away.
    path = [(0, 0, 0), (0, 20, 0)]

    # Should be valid if we ignore endpoints
    assert (
        check_wire_clearance(
            path, assembly, clearance_mm=2.0, ignore_endpoints_dist=15.0
        )
        is True
    )
    # Should fail if we don't
    assert (
        check_wire_clearance(
            path, assembly, clearance_mm=2.0, ignore_endpoints_dist=0.0
        )
        is False
    )


def test_route_wire_helper():
    """T012: Verify route_wire returns correct config."""
    cfg = route_wire(
        wire_id="w1",
        from_comp="psu",
        from_term="v+",
        to_comp="m1",
        to_term="+",
        gauge_awg=18,
        waypoints=[(0, 0, 0), (10, 0, 0)],
        routed_in_3d=True,
    )

    assert cfg.wire_id == "w1"
    assert cfg.length_mm == 10.0
    assert cfg.gauge_awg == 18
    assert cfg.from_terminal.component == "psu"
    assert cfg.routed_in_3d is True


if __name__ == "__main__":
    pytest.main([__file__])
