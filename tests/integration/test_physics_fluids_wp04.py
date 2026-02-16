from unittest.mock import MagicMock, PropertyMock, patch

import numpy as np
import pytest

from shared.models.schemas import (
    BoundingBox,
    Constraints,
    FlowRateObjective,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
    PhysicsConfig,
)
from shared.simulation.backends import (
    SimulationScene,
    StepResult,
)
from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.genesis_backend import GenesisBackend
from worker.simulation.loop import SimulationLoop


@pytest.fixture
def genesis_backend():
    with patch("genesis.init"), patch("genesis.Scene"):
        backend = GenesisBackend()
        # Mock scene and its build status
        mock_scene = MagicMock()
        type(mock_scene).is_built = PropertyMock(return_value=False)
        backend.scene = mock_scene
        yield backend


def test_flow_rate_integration(genesis_backend, tmp_path):
    # Mock scene build status
    type(genesis_backend.scene).is_built = PropertyMock(return_value=True)

    # 1. Define objectives with flow rate
    # Gate plane: center (0, 0, 0), normal (0, 0, 1) - horizontal plane
    flow_obj = FlowRateObjective(
        fluid_id="water",
        gate_plane_point=(0, 0, 0),
        gate_plane_normal=(0, 0, 1),
        target_rate_l_per_s=0.01,  # 10ml/s
        tolerance=0.2,
    )

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10, 10, 10), max=(11, 11, 11)),
            fluid_objectives=[flow_obj],
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

    # 2. Setup mock particles: moving across Z=0 plane
    # 10 particles crossing per step.

    def mock_get_particles():
        # particles start below plane
        if mock_step.time == 0:
            return np.array([[0, 0, -0.1]] * 100)
        # move above plane
        return np.array([[0, 0, 0.1]] * 100)

    # 3. Setup Loop with mocked backend
    with patch("worker.simulation.loop.get_physics_backend") as mock_get:
        mock_get.return_value = genesis_backend

        # Ensure methods are mocks
        genesis_backend.step = MagicMock()
        genesis_backend.get_particle_positions = MagicMock()
        genesis_backend.get_particle_positions.side_effect = mock_get_particles
        genesis_backend.get_all_body_names = MagicMock(return_value=["world"])

        xml_path = tmp_path / "scene.json"
        xml_path.write_text("{}")  # Dummy JSON

        def mock_step(dt):
            mock_step.time += dt
            return StepResult(time=mock_step.time, success=True)

        mock_step.time = 0.0
        genesis_backend.step.side_effect = mock_step

        loop = SimulationLoop(
            str(xml_path),
            backend_type=SimulatorBackendType.GENESIS,
            objectives=objectives,
        )

        # Run for 0.1 seconds (50 steps)
        metrics = loop.step({}, duration=0.1)

        # 4. Verify results
        assert metrics.success is True
        assert len(metrics.fluid_metrics) == 1
        assert metrics.fluid_metrics[0].fluid_id == "water"
        assert metrics.fluid_metrics[0].metric_type == "flow_rate"
        assert metrics.fluid_metrics[0].passed is True
        # measured_rate should be ~5.0 (0.5L / 0.1s)
        assert metrics.fluid_metrics[0].measured_value > 0


def test_gpu_oom_retry_logic(genesis_backend, tmp_path):
    # Mock _load_scene_internal to fail with OOM once, then succeed
    with patch.object(GenesisBackend, "_load_scene_internal") as mock_load:
        mock_load.side_effect = [
            Exception("CUDA error: out of memory"),
            None,  # Success on second attempt
        ]

        scene = SimulationScene(scene_path=str(tmp_path / "scene.json"))
        tmp_path.joinpath("scene.json").write_text("{}")

        with patch("shared.observability.events.emit_event") as mock_emit:
            genesis_backend.load_scene(scene)

            assert mock_load.call_count == 2
            assert genesis_backend.current_particle_multiplier == 0.75

            # Check if event was emitted

            emitted_events = [args[0] for args, _ in mock_emit.call_args_list]
            assert any(e.event_type == "gpu_oom_retry" for e in emitted_events)


def test_electronics_fluid_damage_logic(genesis_backend, tmp_path):
    # Mock scene build status
    type(genesis_backend.scene).is_built = PropertyMock(return_value=True)

    # Setup electronics entity
    genesis_backend.entities = {"controller": MagicMock()}
    genesis_backend.entity_configs = {"controller": {"is_electronics": True}}

    # Mock particles touching electronics
    # distance check in backend: dist < 0.05
    # particles at (0,0,0), controller at (0,0,0)
    particles = np.array([[0.01, 0.01, 0.01]])
    genesis_backend.get_particle_positions = MagicMock(return_value=particles)

    mock_entity_state = MagicMock()
    # state.pos[0] should be [N, 3]
    mock_entity_state.pos = MagicMock()
    mock_entity_state.pos.__getitem__.return_value.cpu.return_value.numpy.return_value = np.array(
        [[0, 0, 0]]
    )
    mock_entity_state.pos.ndim = 3

    genesis_backend.entities["controller"].get_state.return_value = mock_entity_state

    # Run step
    genesis_backend.current_time = 0.0
    res = genesis_backend.step(0.002)

    assert res.success is False
    assert "ELECTRONICS_FLUID_DAMAGE:controller" in res.failure_reason
