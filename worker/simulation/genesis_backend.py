import numpy as np

try:
    import genesis as gs
except ImportError:
    gs = None

from shared.simulation.backends import (
    PhysicsBackend,
    SimulationScene,
    StepResult,
    BodyState,
    SiteState,
    ActuatorState,
    StressField,
    ContactForce,
)


class GenesisBackend(PhysicsBackend):
    def __init__(self):
        self.scene = None
        self.entities = {}
        if gs is not None:
            gs.init(backend=gs.gpu)  # Default to GPU if available

    def load_scene(self, scene: SimulationScene) -> None:
        if gs is None:
            raise ImportError("Genesis not installed")

        self.scene = gs.Scene(
            show_viewer=False,
            vis_options=gs.options.VisOptions(),
        )

        # In Genesis, we build the scene by adding entities
        # For now, this is a placeholder for how we'd map our SimulationScene to Genesis
        # Example:
        # for asset_name, asset_info in scene.assets.items():
        #     if asset_info['type'] == 'mesh':
        #         entity = self.scene.add_entity(gs.morphs.Mesh(file=asset_info['path']))
        #         self.entities[asset_name] = entity

        self.scene.build()

    def step(self, dt: float) -> StepResult:
        if self.scene is None:
            raise RuntimeError("Scene not loaded")

        # Genesis step
        self.scene.step()

        return StepResult(time=0.0, success=True)  # Genesis time tracking needed

    def get_body_state(self, body_id: str) -> BodyState:
        # Placeholder
        return BodyState(
            pos=(0, 0, 0), quat=(1, 0, 0, 0), vel=(0, 0, 0), angvel=(0, 0, 0)
        )

    def get_stress_field(self, body_id: str) -> StressField | None:
        # Genesis FEM support would go here
        return None

    def get_particle_positions(self) -> np.ndarray | None:
        # Genesis MPM support would go here
        return None

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        # Genesis rendering
        return np.zeros((height, width, 3), dtype=np.uint8)

    def get_camera_matrix(self, camera_name: str) -> np.ndarray:
        return np.eye(4)

    def set_site_pos(self, site_name: str, pos: np.ndarray) -> None:
        pass

    def get_contact_forces(self) -> list[ContactForce]:
        return []

    def get_site_state(self, site_name: str) -> SiteState:
        return SiteState(pos=(0, 0, 0), quat=(1, 0, 0, 0), size=(0, 0, 0))

    def get_actuator_state(self, actuator_name: str) -> ActuatorState:
        return ActuatorState(force=0.0, velocity=0.0, ctrl=0.0, forcerange=(0, 0))

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        pass

    def get_all_body_names(self) -> list[str]:
        return []

    def get_all_actuator_names(self) -> list[str]:
        return []

    def get_all_site_names(self) -> list[str]:
        return []

    def check_collision(self, body_name: str, site_name: str) -> bool:
        return False

    def close(self) -> None:
        self.scene = None
