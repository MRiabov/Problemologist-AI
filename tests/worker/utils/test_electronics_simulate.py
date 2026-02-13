import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path
import yaml
from worker.utils.validation import simulate
from build123d import Compound, Box

@pytest.fixture
def dummy_assembly():
    box = Box(10, 10, 10)
    box.label = "part_1"
    box.metadata = {"manufacturing_method": "cnc"}
    return Compound(children=[box])

@pytest.fixture
def mock_workspace(tmp_path):
    # Create objectives.yaml
    obj_data = {
        "objectives": {
            "goal_zone": {"min": [0,0,0], "max": [1,1,1]},
            "build_zone": {"min": [-10,-10,-10], "max": [10,10,10]}
        },
        "simulation_bounds": {"min": [-50,-50,-50], "max": [50,50,50]},
        "moved_object": {"label": "ball", "shape": "sphere", "start_position": [0,0,5], "runtime_jitter": [0,0,0]},
        "constraints": {"max_unit_cost": 100, "max_weight": 5}
    }
    (tmp_path / "objectives.yaml").write_text(yaml.dump(obj_data))
    
    # Create preliminary_cost_estimation.yaml
    cost_data = {
        "constraints": {
            "benchmark_max_unit_cost_usd": 100, "benchmark_max_weight_kg": 5,
            "planner_target_max_unit_cost_usd": 80, "planner_target_max_weight_kg": 4
        },
        "totals": {"estimated_unit_cost_usd": 50, "estimated_weight_g": 1000, "estimate_confidence": "high"},
        "electronics": {
            "power_supply": {"voltage_dc": 24.0, "max_current_a": 10.0},
            "components": [
                {"component_id": "psu", "type": "power_supply", "cots_part_id": "DR-120-24"},
                {"component_id": "motor_1", "type": "motor", "assembly_part_ref": "part_1", "rated_voltage": 24.0, "stall_current_a": 2.0}
            ],
            "wiring": [{"wire_id": "w1", "from": {"component": "psu", "terminal": "+"}, "to": {"component": "motor_1", "terminal": "+"}, "gauge_awg": 18, "length_mm": 100}]
        }
    }
    (tmp_path / "preliminary_cost_estimation.yaml").write_text(yaml.dump(cost_data))
    return tmp_path

def test_simulate_with_electronics(dummy_assembly, mock_workspace):
    with patch("worker.utils.validation.SimulationBuilder") as mock_builder_cls:
        with patch("worker.utils.validation.SimulationLoop") as mock_loop_cls:
            with patch("worker.utils.validation.prerender_24_views", return_value=[]):
                # Setup mocks
                mock_builder = mock_builder_cls.return_value
                mock_builder.build_from_assembly.return_value = mock_workspace / "scene.xml"
                (mock_workspace / "scene.xml").write_text("<mujoco/>")
                
                mock_loop = mock_loop_cls.return_value
                mock_metrics = MagicMock()
                mock_metrics.success = True
                mock_metrics.fail_reason = None
                mock_loop.step.return_value = mock_metrics
                
                # Mock calculate_assembly_totals to avoid actual DFM calls in this unit test
                with patch("worker.utils.validation.calculate_assembly_totals", return_value=(75.0, 1200.0)):
                    result = simulate(dummy_assembly, output_dir=mock_workspace)
                    
                    assert result.success == True
                    assert result.total_cost == 75.0
                    assert result.total_weight_g == 1200.0
                    
                    # Verify builder was called with electronics
                    mock_builder.build_from_assembly.assert_called_once()
                    call_args = mock_builder.build_from_assembly.call_args[1]
                    assert call_args["electronics"] is not None
                    assert call_args["electronics"].power_supply.voltage_dc == 24.0
