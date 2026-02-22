import sys
import os
import pytest
import importlib.util
from build123d import Compound

# Import script.py using importlib to avoid sys.path issues and name conflicts
script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../worker_light/agent_files/script.py'))

spec = importlib.util.spec_from_file_location("agent_script", script_path)
script = importlib.util.module_from_spec(spec)
sys.modules["agent_script"] = script
spec.loader.exec_module(script)

def test_build_determinism():
    """Test that build produces the same result for the same seed."""
    part1 = script.build(seed=42, scale=1.0)
    part2 = script.build(seed=42, scale=1.0)
    assert part1.volume == pytest.approx(part2.volume)
    assert part1.bounding_box().size.X == pytest.approx(part2.bounding_box().size.X)

def test_build_randomization():
    """Test that build produces different results for different seeds."""
    part1 = script.build(seed=1, scale=1.0)
    part2 = script.build(seed=2, scale=1.0)

    # It's theoretically possible but highly unlikely that random dimensions match exactly
    assert part1.volume != part2.volume

def test_build_scale():
    """Test that scale affects the dimensions."""
    part1 = script.build(seed=10, scale=1.0)
    part2 = script.build(seed=10, scale=2.0)

    # Volume should scale by cube of linear scale (2^3 = 8)
    assert part2.volume == pytest.approx(part1.volume * 8.0, rel=1e-5)
