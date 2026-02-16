from shared.models.simulation import (
    FluidMetricResult,
    SimulationMetrics,
    SimulationResult,
    StressSummary,
)


def test_stress_summary_validation():
    data = {
        "part_label": "bracket",
        "max_von_mises_pa": 100.0,
        "mean_von_mises_pa": 50.0,
        "safety_factor": 2.0,
        "location_of_max": [0.0, 1.0, 2.0],
        "utilization_pct": 50.0,
    }
    summary = StressSummary(**data)
    assert summary.part_label == "bracket"
    assert summary.location_of_max == (0.0, 1.0, 2.0)


def test_fluid_metric_validation():
    data = {
        "metric_type": "fluid_containment",
        "fluid_id": "water",
        "measured_value": 0.95,
        "target_value": 0.90,
        "passed": True,
    }
    metric = FluidMetricResult(**data)
    assert metric.passed is True


def test_simulation_metrics_defaults():
    metrics = SimulationMetrics(
        total_time=10.0, total_energy=100.0, max_velocity=1.0, success=True
    )
    assert metrics.stress_summaries == []
    assert metrics.fluid_metrics == []
    assert metrics.confidence == "high"


def test_simulation_result_serialization():
    result = SimulationResult(success=True, summary="Success", total_cost=10.0)
    dump = result.model_dump()
    assert dump["success"] is True
    assert dump["total_cost"] == 10.0
    assert "render_paths" in dump
    assert dump["render_paths"] == []

    # Test round-trip
    restored = SimulationResult.model_validate(dump)
    assert restored.success == result.success
    assert restored.total_cost == result.total_cost
