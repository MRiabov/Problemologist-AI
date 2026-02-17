import yaml

from shared.models.simulation import StressSummary
from worker.utils.validation import (
    SimulationResult,
    define_fluid,
    get_stress_report,
    save_simulation_result,
    set_soft_mesh,
)


def test_get_stress_report_with_advice(tmp_path, monkeypatch):
    # Setup mock simulation result
    summary = StressSummary(
        part_label="bracket",
        max_von_mises_pa=100e6,
        mean_von_mises_pa=50e6,
        safety_factor=1.4,  # Should trigger WARNING
        location_of_max=(0, 0, 0),
        utilization_pct=70.0,
    )
    res = SimulationResult(success=True, summary="OK", stress_summaries=[summary])

    res_path = tmp_path / "simulation_result.json"
    save_simulation_result(res, res_path)

    # Mock working directory
    monkeypatch.setenv("RENDERS_DIR", str(tmp_path / "renders"))
    (tmp_path / "renders").mkdir()

    # Move simulation_result.json to where it's expected (renders/../)
    # The tool looks in Path("simulation_result.json") and RENDERS_DIR/../simulation_result.json
    res_path_target = tmp_path / "simulation_result.json"
    save_simulation_result(res, res_path_target)

    # We need to change the current working directory for the test or use a mock
    import os

    orig_cwd = os.getcwd()
    os.chdir(tmp_path)
    try:
        report = get_stress_report("bracket")
        assert report is not None
        assert "advice" in report
        assert "Safety factor low" in report["advice"]
    finally:
        os.chdir(orig_cwd)


def test_define_fluid_updates_yaml(tmp_path, monkeypatch):
    obj_path = tmp_path / "objectives.yaml"
    # Basic objectives.yaml
    data = {
        "objectives": {
            "goal_zone": {"min": [0, 0, 0], "max": [1, 1, 1]},
            "build_zone": {"min": [-10, -10, -10], "max": [10, 10, 10]},
            "forbid_zones": [],
        },
        "simulation_bounds": {"min": [-50, -50, 0], "max": [50, 50, 100]},
        "moved_object": {
            "label": "ball",
            "shape": "sphere",
            "start_position": [0, 0, 5],
            "runtime_jitter": [0, 0, 0],
        },
        "constraints": {"max_unit_cost": 100, "max_weight": 10},
        "physics": {"backend": "mujoco"},
    }
    obj_path.write_text(yaml.dump(data))

    monkeypatch.setenv("RENDERS_DIR", str(tmp_path / "renders/dummy.png"))
    (tmp_path / "renders").mkdir()

    define_fluid("water", "box", (0, 0, 0), size=(0.1, 0.1, 0.1), output_dir=tmp_path)

    updated_data = yaml.safe_load(obj_path.read_text())
    assert "fluids" in updated_data
    assert len(updated_data["fluids"]) == 1
    assert updated_data["fluids"][0]["fluid_id"] == "water"


def test_set_soft_mesh(tmp_path, monkeypatch):
    obj_path = tmp_path / "objectives.yaml"
    data = {
        "objectives": {
            "goal_zone": {"min": [0, 0, 0], "max": [1, 1, 1]},
            "build_zone": {"min": [-10, -10, -10], "max": [10, 10, 10]},
        },
        "simulation_bounds": {"min": [-50, -50, 0], "max": [50, 50, 100]},
        "moved_object": {
            "label": "ball",
            "shape": "sphere",
            "start_position": [0, 0, 5],
            "runtime_jitter": [0, 0, 0],
        },
        "constraints": {"max_unit_cost": 100, "max_weight": 10},
        "physics": {"backend": "mujoco", "fem_enabled": False},
    }
    obj_path.write_text(yaml.dump(data))

    set_soft_mesh("bracket", enabled=True, output_dir=tmp_path)

    updated_data = yaml.safe_load(obj_path.read_text())
    assert updated_data["physics"]["fem_enabled"] is True
    assert updated_data["physics"]["backend"] == "genesis"
