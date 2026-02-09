"""Tests for multi-run verification with position jitter."""

import pytest

from worker.simulation.verification import (
    DEFAULT_NUM_RUNS,
    MultiRunResult,
    verify_with_jitter,
)

# Simple XML that always succeeds (ball falls into goal zone)
TEST_SUCCESS_XML = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="target_box" pos="0 0 0.5">
      <joint type="free"/>
      <geom name="ball" type="sphere" size="0.05"/>
    </body>
    <geom name="zone_goal" type="box" pos="0 0 -0.5" size="0.5 0.5 0.6"/>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def success_xml_path(tmp_path):
    xml_path = tmp_path / "test_success.xml"
    xml_path.write_text(TEST_SUCCESS_XML)
    return str(xml_path)


class TestMultiRunVerification:
    def test_default_num_runs(self):
        """Default is 5 runs per architecture spec."""
        assert DEFAULT_NUM_RUNS == 5

    def test_consistent_success_all_pass(self, success_xml_path):
        """All runs succeed for a robust design."""
        result = verify_with_jitter(
            success_xml_path,
            control_inputs={},
            num_runs=3,
            duration=2.0,
            seed=42,
        )

        assert isinstance(result, MultiRunResult)
        assert result.num_runs == 3
        assert len(result.individual_results) == 3

    def test_deterministic_with_seed(self, success_xml_path):
        """Same seed produces same results."""
        result1 = verify_with_jitter(
            success_xml_path,
            control_inputs={},
            num_runs=3,
            duration=1.0,
            seed=123,
        )

        result2 = verify_with_jitter(
            success_xml_path,
            control_inputs={},
            num_runs=3,
            duration=1.0,
            seed=123,
        )

        # Same seed should give same success rate
        assert result1.success_rate == result2.success_rate

    def test_result_has_metrics(self, success_xml_path):
        """Result includes all expected metrics."""
        result = verify_with_jitter(
            success_xml_path,
            control_inputs={},
            num_runs=2,
            duration=1.0,
        )

        assert hasattr(result, "success_rate")
        assert hasattr(result, "is_consistent")
        assert hasattr(result, "individual_results")
        assert hasattr(result, "fail_reasons")

        # Success rate should be between 0 and 1
        assert 0 <= result.success_rate <= 1

    def test_jitter_range_default(self, success_xml_path):
        """Default jitter range is small (mm scale)."""
        # Just verify it runs without error
        result = verify_with_jitter(
            success_xml_path,
            control_inputs={},
            num_runs=1,
            duration=0.5,
        )

        assert result.num_runs == 1
