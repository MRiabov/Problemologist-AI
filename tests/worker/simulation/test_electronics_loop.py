import pytest
from unittest.mock import MagicMock, patch
from worker.simulation.loop import SimulationLoop
from shared.models.schemas import ElectronicsSection, PowerSupplyConfig, ElectronicComponent, WireConfig
from shared.simulation.backends import SimulatorBackendType

@pytest.fixture
def mock_backend():
    backend = MagicMock()
    backend.get_all_site_names.return_value = []
    backend.get_all_body_names.return_value = []
    backend.get_all_actuator_names.return_value = ["motor_1"]
    backend.step.return_value = MagicMock(time=0.002, success=True)
    backend.get_actuator_state.return_value = MagicMock(ctrl=0.0, velocity=0.0, force=0.0, forcerange=(0,0))
    return backend

def test_simulation_loop_power_gating(mock_backend):
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0),
        components=[
            ElectronicComponent(component_id="psu", type="power_supply"),
            ElectronicComponent(component_id="motor_1_comp", type="motor", assembly_part_ref="motor_1", rated_voltage=24.0, stall_current_a=2.0)
        ],
        wiring=[
            WireConfig(wire_id="w1", from_terminal={"component": "psu", "terminal": "+"}, to_terminal={"component": "motor_1_comp", "terminal": "+"}, gauge_awg=18, length_mm=100)
        ]
    )
    
    with patch("worker.simulation.loop.get_physics_backend", return_value=mock_backend):
        # We also need to mock PySpice validation to avoid the 'run' failed error during init
        with patch("shared.pyspice_utils.validate_circuit") as mock_validate:
            # Mock powered state (True)
            mock_validate.return_value = MagicMock(
                valid=True, 
                node_voltages={"motor_1_comp_+": 24.0, "motor_1_comp_-": 0.0},
                branch_currents={},
                total_draw_a=1.0,
                errors=[]
            )
            
            loop = SimulationLoop(
                xml_path="dummy.xml",
                electronics=electronics,
                backend_type=SimulatorBackendType.MUJOCO
            )
            
            # 1. Step with power
            dynamic_controllers = {"motor_1": lambda t: 1.0}
            loop.step(control_inputs={}, duration=0.002, dynamic_controllers=dynamic_controllers)
            
            # Check that apply_control was called with 1.0
            mock_backend.apply_control.assert_any_call({"motor_1": 1.0})
            
            # 2. Mock unpowered state
            mock_validate.return_value.node_voltages = {"motor_1_comp_+": 0.0, "motor_1_comp_-": 0.0}
            loop = SimulationLoop(
                xml_path="dummy.xml",
                electronics=electronics,
                backend_type=SimulatorBackendType.MUJOCO
            )
            loop.step(control_inputs={}, duration=0.002, dynamic_controllers=dynamic_controllers)
            
            # Check that apply_control was called with 0.0
            mock_backend.apply_control.assert_any_call({"motor_1": 0.0})

def test_wire_torn_failure(mock_backend):
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0),
        wiring=[
            WireConfig(wire_id="w1", from_terminal={"component": "psu", "terminal": "+"}, to_terminal={"component": "m1", "terminal": "+"}, gauge_awg=18, length_mm=100, routed_in_3d=True)
        ]
    )
    
    with patch("worker.simulation.loop.get_physics_backend", return_value=mock_backend):
        with patch("shared.pyspice_utils.validate_circuit") as mock_validate:
            mock_validate.return_value = MagicMock(valid=True, node_voltages={}, errors=[])
            
            loop = SimulationLoop(xml_path="dummy.xml", electronics=electronics)
            
            # Mock high tension
            mock_backend.get_tendon_tension.return_value = 1000.0 # Way above limit
            
            metrics = loop.step(control_inputs={}, duration=0.01)
            
            assert metrics.success == False
            assert "FAILED_WIRE_TORN" in metrics.fail_reason
            # Power gating should set motors to 0 after tear
            assert loop.is_powered_map == {} or all(v == 0.0 for v in loop.is_powered_map.values())
