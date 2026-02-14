import pytest
import numpy as np
from unittest.mock import MagicMock, patch, PropertyMock
from shared.simulation.backends import (
    BodyState,
    SimulatorBackendType,
    StepResult,
    StressField,
)
from worker.simulation.loop import SimulationLoop
from shared.models.schemas import (
    ObjectivesYaml,
    ObjectivesSection,
    BoundingBox,
    MovedObject,
    Constraints,
    FluidDefinition,
    FluidVolume,
    FluidContainmentObjective,
    PhysicsConfig,
)

# Shared mock to avoid "already initialized" errors if multiple backends are created
_SHARED_BACKEND_MOCK = MagicMock()


@pytest.fixture
def mock_genesis_backend():
    with patch("worker.simulation.loop.get_physics_backend") as mock_get:
        backend = _SHARED_BACKEND_MOCK
        mock_get.return_value = backend

        # Reset mock for each test
        backend.reset_mock()
        backend.get_particle_positions.return_value = None
        backend.get_stress_field.return_value = None
        backend.get_stress_field.side_effect = None
        backend.get_particle_positions.side_effect = None

        # Default behavior: success on step
        backend.step.return_value = StepResult(time=0.002, success=True)
        backend.get_all_body_names.return_value = ["world", "bucket"]
        backend.get_body_state.return_value = BodyState(
            pos=(0, 0, 0), quat=(1, 0, 0, 0), vel=(0, 0, 0), angvel=(0, 0, 0)
        )
        backend.get_all_site_names.return_value = []
        backend.get_all_actuator_names.return_value = []
        backend.get_all_tendon_names.return_value = []

        # Mock scene and its build status
        mock_scene = MagicMock()
        type(mock_scene).is_built = PropertyMock(return_value=True)
        backend.scene = mock_scene

        # Default stress field
        backend.get_stress_field.return_value = StressField(
            nodes=np.zeros((1, 3)),
            stress=np.array([100e6]),  # 100 MPa
        )

        yield backend


def test_fluid_containment_integration(mock_genesis_backend, tmp_path):
    # 1. Define objectives with fluid containment
    # Containment zone: [0, 0, 0] to [1, 1, 1]
    containment_obj = FluidContainmentObjective(
        fluid_id="water",
        containment_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
        threshold=0.9,
        eval_at="end",
    )

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
            fluid_objectives=[containment_obj],
            build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        ),
        physics=PhysicsConfig(backend="genesis"),
        simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        moved_object=MovedObject(
            label="target",
            start_position=(0, 0, 0),
            runtime_jitter=(0, 0, 0),
            shape="box",
        ),
        constraints=Constraints(max_unit_cost=100, max_weight=10),
        fluids=[
            FluidDefinition(
                fluid_id="water",
                initial_volume=FluidVolume(
                    type="box", center=(0.5, 0.5, 0.5), size=(0.1, 0.1, 0.1)
                ),
            )
        ],
    )

    # 2. Setup mock particles: 100 particles inside the zone
    inside_particles = np.random.uniform(0.1, 0.9, (100, 3))
    mock_genesis_backend.get_particle_positions.return_value = inside_particles
    mock_genesis_backend.get_stress_field.return_value = StressField(
        nodes=np.zeros((1, 3)), stress=np.array([100e6])
    )

    # 3. Run simulation loop
    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco/>")  # Dummy

    loop = SimulationLoop(
        str(xml_path),
        backend_type=SimulatorBackendType.GENESIS,
        objectives=objectives,
        smoke_test_mode=True,
    )

    # Mocking current_time increment in loop manually is hard,
    # but we can mock backend.step to advance time.
    def mock_step(dt):
        mock_step.time += dt
        return StepResult(time=mock_step.time, success=True)

    mock_step.time = 0.0
    mock_genesis_backend.step.side_effect = mock_step

    # Run for 0.1 seconds (50 steps at dt=0.002)
    metrics = loop.step({}, duration=0.1)

    # 4. Verify results
    assert metrics.success is True, f"Simulation failed: {metrics.fail_reason}"
    assert len(metrics.fluid_metrics) == 1
    assert metrics.fluid_metrics[0].fluid_id == "water"
    assert metrics.fluid_metrics[0].passed is True
    assert metrics.fluid_metrics[0].measured_value == 1.0


def test_fluid_containment_failure(mock_genesis_backend, tmp_path):
    # 1. Define objectives
    containment_obj = FluidContainmentObjective(
        fluid_id="water",
        containment_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
        threshold=0.9,
    )

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
            fluid_objectives=[containment_obj],
            build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        ),
        physics=PhysicsConfig(backend="genesis"),
        simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        moved_object=MovedObject(
            label="target",
            start_position=(0, 0, 0),
            runtime_jitter=(0, 0, 0),
            shape="box",
        ),
        constraints=Constraints(max_unit_cost=100, max_weight=10),
    )

    # 2. Setup mock particles: 50 inside, 50 outside (ratio 0.5 < 0.9)
    inside = np.random.uniform(0.1, 0.9, (50, 3))
    outside = np.random.uniform(2.0, 3.0, (50, 3))
    mock_genesis_backend.get_particle_positions.return_value = np.vstack(
        [inside, outside]
    )
    mock_genesis_backend.get_stress_field.return_value = StressField(
        nodes=np.zeros((1, 3)), stress=np.array([100e6])
    )

    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco/>")

    loop = SimulationLoop(
        str(xml_path),
        backend_type=SimulatorBackendType.GENESIS,
        objectives=objectives,
        smoke_test_mode=True,
    )

    def mock_step(dt):
        mock_step.time += dt
        return StepResult(time=mock_step.time, success=True)

    mock_step.time = 0.0
    mock_genesis_backend.step.side_effect = mock_step

    # Run simulation
    metrics = loop.step({}, duration=0.1)

    # 3. Verify failure
    assert metrics.success is False
    assert metrics.fail_reason is not None
    assert "FAILED_FLUID_OBJECTIVE:water" in metrics.fail_reason
    assert metrics.fluid_metrics[0].passed is False
    assert metrics.fluid_metrics[0].measured_value == 0.5


def test_part_breakage_integration(mock_genesis_backend, tmp_path):
    # Setup mock stress field that exceeds ultimate stress (default 310 MPa)
    mock_genesis_backend.get_stress_field.return_value = StressField(
        nodes=np.zeros((1, 3)),
        stress=np.array([400e6]),  # 400 MPa
    )

    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco/>")

    loop = SimulationLoop(
        str(xml_path), backend_type=SimulatorBackendType.GENESIS, smoke_test_mode=True
    )

    def mock_step(dt):
        mock_step.time += dt
        return StepResult(time=mock_step.time, success=True)

    mock_step.time = 0.0
    mock_genesis_backend.step.side_effect = mock_step

    metrics = loop.step({}, duration=0.1)

    assert metrics.success is False
    assert metrics.fail_reason is not None
    assert "FAILED_PART_BREAKAGE:world" in metrics.fail_reason
