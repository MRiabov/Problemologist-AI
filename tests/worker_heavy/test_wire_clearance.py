from unittest.mock import MagicMock, patch

from build123d import Box

from shared.enums import ElectronicComponentType
from shared.models.schemas import (
    ElectronicComponent,
    ElectronicsSection,
    PowerSupplyConfig,
    WireConfig,
    WireTerminal,
)
from worker_heavy.simulation.loop import SimulationLoop


@patch("worker_heavy.simulation.loop.get_physics_backend")
@patch("shared.pyspice_utils.validate_circuit")
def test_wire_clearance_violation(mock_validate_circuit, mock_get_backend):
    # Setup mock backend
    mock_backend = MagicMock()
    mock_get_backend.return_value = mock_backend
    mock_backend.get_all_site_names.return_value = []
    mock_backend.get_all_actuator_names.return_value = []
    mock_backend.get_all_body_names.return_value = []

    # 1. Create a Box centered at origin (10x10x10)
    # This box occupies -5 to +5 in X, Y, Z
    component = Box(10, 10, 10)

    # 2. Define a wire that goes through the box
    # From (-10, 0, 0) to (10, 0, 0). Passes through (-5, 0, 0) to (5, 0, 0).
    wire = WireConfig(
        wire_id="wire_1",
        from_terminal=WireTerminal(component="src", terminal="t1"),
        to_terminal=WireTerminal(component="dst", terminal="t1"),
        gauge_awg=20,
        length_mm=20.0,
        waypoints=[(-10.0, 0.0, 0.0), (10.0, 0.0, 0.0)],
        routed_in_3d=True,
    )

    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0),
        wiring=[wire],
        components=[
            ElectronicComponent(
                component_id="src", type=ElectronicComponentType.CONNECTOR
            ),
            ElectronicComponent(
                component_id="dst", type=ElectronicComponentType.CONNECTOR
            ),
        ],
    )

    # Mock validate_circuit to avoid libngspice dependency and ensure valid circuit
    mock_validate_circuit.return_value = MagicMock(
        valid=True, errors=[], node_voltages={}
    )

    # 3. Initialize SimulationLoop
    # Note: We need to mock validate_and_price because it requires config loading
    with patch("worker_heavy.simulation.loop.validate_and_price") as mock_validate:
        mock_validate.return_value = MagicMock(is_manufacturable=True)
        # Mock load_config as well since it's called if custom config exists or fallback
        with patch("worker_heavy.simulation.loop.load_config"):
            loop = SimulationLoop(
                xml_path="dummy.xml",
                component=component,
                electronics=electronics,
                smoke_test_mode=True,
            )

            # 4. Check if initialization detected the error
            assert loop.wire_clearance_error is not None
            assert "Wire clearance violation detected" in loop.wire_clearance_error

            # 5. Run step and verify fail_reason
            metrics = loop.step(control_inputs={})
            assert metrics.success is False
            from shared.enums import FailureReason

            assert metrics.failure.reason == FailureReason.VALIDATION_FAILED
            # Ensure the specific error is part of the detail message
            assert loop.wire_clearance_error in metrics.failure.detail


@patch("worker_heavy.simulation.loop.get_physics_backend")
@patch("shared.pyspice_utils.validate_circuit")
def test_wire_clearance_success(mock_validate_circuit, mock_get_backend):
    # Mock validate_circuit
    mock_validate_circuit.return_value = MagicMock(
        valid=True, errors=[], node_voltages={}
    )

    # Setup mock backend
    mock_backend = MagicMock()
    mock_get_backend.return_value = mock_backend
    mock_backend.get_all_site_names.return_value = []
    mock_backend.get_all_actuator_names.return_value = []
    mock_backend.get_all_body_names.return_value = []

    # 1. Create a Box
    component = Box(10, 10, 10)

    # 2. Define a wire that goes around the box (safe)
    # (-10, 0, 0) -> (-10, 20, 0) -> (10, 20, 0) -> (10, 0, 0)
    # Box is at Y=0 (width 10, so -5 to 5). Wire at Y=20 is safe (distance 15mm > 2mm clearance)
    wire = WireConfig(
        wire_id="wire_safe",
        from_terminal=WireTerminal(component="src", terminal="t1"),
        to_terminal=WireTerminal(component="dst", terminal="t1"),
        gauge_awg=20,
        length_mm=100.0,
        waypoints=[
            (-10.0, 0.0, 0.0),
            (-10.0, 20.0, 0.0),
            (10.0, 20.0, 0.0),
            (10.0, 0.0, 0.0),
        ],
        routed_in_3d=True,
    )

    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0),
        wiring=[wire],
        components=[],
    )

    with patch("worker_heavy.simulation.loop.validate_and_price") as mock_validate:
        mock_validate.return_value = MagicMock(is_manufacturable=True)
        with patch("worker_heavy.simulation.loop.load_config"):
            loop = SimulationLoop(
                xml_path="dummy.xml",
                component=component,
                electronics=electronics,
                smoke_test_mode=True,
            )

            # 4. Check if initialization passed
            assert loop.wire_clearance_error is None
