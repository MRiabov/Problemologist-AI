import os

import yaml

from worker.simulation.loop import StressSummary
from worker.utils.validation import (
    SimulationResult,
    define_fluid,
    get_stress_report,
    set_soft_mesh,
)


def test_get_stress_report_logic():
    # Setup mock simulation result
    summaries = [
        StressSummary(
            part_label="part1",
            max_von_mises_pa=100e6,
            mean_von_mises_pa=50e6,
            safety_factor=2.0,
            location_of_max=(0, 0, 0),
            utilization_pct=50.0,
        ),
        StressSummary(
            part_label="part1",
            max_von_mises_pa=250e6,
            mean_von_mises_pa=100e6,
            safety_factor=1.1,
            location_of_max=(1, 1, 1),
            utilization_pct=90.0,
        ),
    ]

    result = SimulationResult(
        success=True, summary="Test summary", stress_summaries=summaries
    )

    # Manually set the global variable for testing
    import worker.utils.validation as validation

    validation.LAST_SIMULATION_RESULT = result

    # Should return the worst case (safety_factor=1.1)
    report = get_stress_report("part1")
    assert report is not None
    assert report["safety_factor"] == 1.1
    assert "critical" in report["advice"].lower()

    # Test another range
    summaries[1].safety_factor = 1.4
    report = get_stress_report("part1")
    assert "low" in report["advice"].lower()

    summaries[1].safety_factor = 3.0
    summaries[0].safety_factor = 2.0
    report = get_stress_report("part1")
    assert "acceptable" in report["advice"].lower()


def test_define_fluid(tmp_path):
    os.environ["RENDERS_DIR"] = str(tmp_path / "renders")
    (tmp_path / "renders").mkdir()

    obj_path = tmp_path / "objectives.yaml"
    # Basic objectives.yaml content
    obj_path.write_text("""
objectives:
  goal_zone: {min: [0,0,0], max: [1,1,1]}
  build_zone: {min: [0,0,0], max: [1,1,1]}
simulation_bounds: {min: [0,0,0], max: [1,1,1]}
moved_object:
  label: x
  shape: sphere
  start_position: [0,0,0]
  runtime_jitter: [0,0,0]
constraints: {max_unit_cost: 0, max_weight: 0}
""")

    fluid = define_fluid(
        name="water",
        shape_type="box",
        center=(0, 0, 0),
        size=(10, 10, 10),
        output_dir=tmp_path,
    )

    assert fluid["fluid_id"] == "water"
    assert obj_path.exists()

    with obj_path.open() as f:
        data = yaml.safe_load(f)
        assert "fluids" in data
        assert len(data["fluids"]) == 1
        assert data["fluids"][0]["fluid_id"] == "water"


def test_set_soft_mesh(tmp_path):
    os.environ["RENDERS_DIR"] = str(tmp_path / "renders")
    (tmp_path / "renders").mkdir()

    obj_path = tmp_path / "objectives.yaml"
    obj_path.write_text("""
objectives:
  goal_zone: {min: [0,0,0], max: [1,1,1]}
  build_zone: {min: [0,0,0], max: [1,1,1]}
simulation_bounds: {min: [0,0,0], max: [1,1,1]}
moved_object:
  label: x
  shape: sphere
  start_position: [0,0,0]
  runtime_jitter: [0,0,0]
constraints: {max_unit_cost: 0, max_weight: 0}
""")

    res = set_soft_mesh("part1", enabled=True, output_dir=tmp_path)
    assert res is True

    with obj_path.open() as f:
        data = yaml.safe_load(f)
        assert data["physics"]["fem_enabled"] is True
        assert data["physics"]["backend"] == "genesis"
