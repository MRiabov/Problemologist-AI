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
        # Defer initialization until implemented

    def load_scene(self, scene: SimulationScene) -> None:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def step(self, dt: float) -> StepResult:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_body_state(self, body_id: str) -> BodyState:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_stress_field(self, body_id: str) -> StressField | None:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_particle_positions(self) -> np.ndarray | None:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_camera_matrix(self, camera_name: str) -> np.ndarray:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def set_site_pos(self, site_name: str, pos: np.ndarray) -> None:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_contact_forces(self) -> list[ContactForce]:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_site_state(self, site_name: str) -> SiteState:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_actuator_state(self, actuator_name: str) -> ActuatorState:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_all_body_names(self) -> list[str]:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_all_actuator_names(self) -> list[str]:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def get_all_site_names(self) -> list[str]:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def check_collision(self, body_name: str, site_name: str) -> bool:
        raise NotImplementedError("Genesis backend is not yet implemented.")

    def close(self) -> None:
        self.scene = None
