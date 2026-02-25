from build123d import Box, Location

from worker_heavy.workbenches.config import load_config
from worker_heavy.workbenches.injection_molding import analyze_im


def test_analyze_im_basic_box():
    """Test analysis on a simple moldable box."""
    # A box is perfectly moldable from Z direction if we don't care about side draft for now
    # or if we provide a configuration that accepts it.
    box = Box(10, 10, 2)
    config = load_config()

    # Force a lenient config for the basic box if needed, or rely on defaults
    result = analyze_im(box, config)

    # A simple box with vertical walls might fail 2.0 deg draft check
    # But let's see what happens.
    assert result is not None
    assert hasattr(result, "is_manufacturable")
    assert hasattr(result, "unit_cost")
    assert result.unit_cost > 0


def test_analyze_im_undercut():
    """Test detection of undercuts."""
    # Create a part with a clear undercut from both Z directions
    # A large box with a smaller box inside it (a cavity with a overhang)
    # Actually, a simple shelf is an undercut from +Z.
    # For IM it must be occluded from BOTH.
    # An 'I' beam on its side.
    base = Box(10, 10, 2)
    vertical = Box(2, 10, 10).moved(Location((0, 0, 6)))
    top = Box(10, 10, 2).moved(Location((0, 0, 12)))
    part = base + vertical + top

    # Orientation: pulled along Z. The center part of the 'I' is an undercut.

    config = load_config()
    result = analyze_im(part, config)

    undercut_found = any(
        "occluded" in v.lower() or "undercut" in v.lower() for v in result.violations
    )
    assert undercut_found


def test_analyze_im_wall_thickness():
    """Test wall thickness validation."""
    # Very thin part
    thin_box = Box(10, 10, 0.1)
    config = load_config()

    result = analyze_im(thin_box, config)

    thin_violation = any("thin" in v.lower() for v in result.violations)
    assert thin_violation


from structlog.testing import capture_logs


def test_im_logging():
    """Verify that IM analysis logs its progress."""
    box = Box(10, 10, 0.1)
    config = load_config()

    with capture_logs() as captured:
        analyze_im(box, config)

    assert any(log["event"] == "starting_im_analysis" for log in captured)
    assert any(log["event"] == "wall_too_thin" for log in captured)
    assert any(log["event"] == "im_analysis_complete" for log in captured)


def test_im_cost_calculation():
    """Test cost calculation logic."""
    box = Box(10, 10, 2)
    config = load_config()

    result1 = analyze_im(box, config)

    # Cost should be dominated by tooling for low quantity (1 in analyze_im)
    assert result1.unit_cost > 5000.0

    # Test with context for reuse
    from shared.workers.workbench_models import WorkbenchContext
    from worker_heavy.workbenches.injection_molding import _calculate_im_cost

    context = WorkbenchContext()
    cost1 = _calculate_im_cost(box, config, quantity=1, context=context)
    cost2 = _calculate_im_cost(box, config, quantity=1, context=context)

    assert cost2.total_cost < cost1.total_cost
    assert cost2.is_reused is True
