from typing import Any, Protocol, runtime_checkable

import numpy as np
from pydantic import BaseModel, Field

from shared.models.simulation import FluidMetricResult, StressSummary


class BodyState(BaseModel):
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    vel: tuple[float, float, float]
    angvel: tuple[float, float, float]


class StressField(BaseModel):
    nodes: np.ndarray  # (N, 3)
    stress: np.ndarray  # (N,) von Mises stress

    model_config = {"arbitrary_types_allowed": True}


class ContactForce(BaseModel):
    body1: str
    body2: str
    force: tuple[float, float, float]
    position: tuple[float, float, float]


class ActuatorState(BaseModel):
    force: float
    velocity: float
    ctrl: float
    forcerange: tuple[float, float]


class SiteState(BaseModel):
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    size: tuple[float, float, float]


class StepResult(BaseModel):
    time: float
    success: bool
    failure_reason: str | None = None


class SceneConfig(BaseModel):
    """Configuration for the simulation scene."""

    particle_budget: int = 100000
    model_config = {"extra": "allow"}


class SceneAssets(BaseModel):
    """Assets associated with a simulation scene."""

    gs_scene: Any | None = None
    entities: list[dict[str, Any]] = []
    motors: list[dict[str, Any]] = []
    cables: list[dict[str, Any]] = []
    fluids: list[dict[str, Any]] = []
    model_config = {"extra": "allow", "arbitrary_types_allowed": True}


class SimulationScene(BaseModel):
    """Container for scene definition.
    For MuJoCo, this might be the XML path.
    For Genesis, it might be a list of morphs/assets.
    """

    scene_path: str | None = None
    assets: SceneAssets = Field(default_factory=SceneAssets)
    config: SceneConfig = Field(default_factory=SceneConfig)


@runtime_checkable
class PhysicsBackend(Protocol):
    """Interface for physics simulators."""

    def load_scene(self, scene: SimulationScene, render_only: bool = False) -> None: ...

    def step(self, dt: float) -> StepResult: ...
    def get_body_state(self, body_id: str) -> BodyState: ...
    def get_state(self) -> dict[str, Any]: ...
    def get_stress_field(self, body_id: str) -> StressField | None: ...
    def get_max_stress(self) -> float: ...
    def get_stress_summaries(self) -> list[StressSummary]: ...
    def get_particle_positions(self) -> np.ndarray | None: ...
    def get_fluid_metrics(self) -> list[FluidMetricResult]: ...

    # Rendering & Visualization
    def render(self) -> np.ndarray: ...
    def render_camera(
        self, camera_name: str, width: int, height: int
    ) -> np.ndarray: ...

    def set_camera(
        self,
        camera_name: str,
        pos: tuple[float, float, float] | None = None,
        lookat: tuple[float, float, float] | None = None,
        up: tuple[float, float, float] | None = None,
        fov: float | None = None,
    ) -> None: ...

    def get_camera_matrix(self, camera_name: str) -> np.ndarray: ...

    # Advanced interactions (superset of MuJoCo)
    def set_site_pos(self, site_name: str, pos: np.ndarray) -> None: ...
    def get_site_state(self, site_name: str) -> SiteState: ...
    def get_actuator_state(self, actuator_name: str) -> ActuatorState: ...
    def apply_control(self, control_inputs: dict[str, float]) -> None: ...
    def get_contact_forces(self) -> list[ContactForce]: ...

    def get_all_body_names(self) -> list[str]: ...
    def get_all_actuator_names(self) -> list[str]: ...
    def get_all_site_names(self) -> list[str]: ...
    def get_all_tendon_names(self) -> list[str]: ...

    def check_collision(self, body_name: str, site_name: str) -> bool: ...
    def get_tendon_tension(self, tendon_name: str) -> float: ...

    def apply_jitter(
        self, body_name: str, jitter: tuple[float, float, float]
    ) -> None: ...

    def set_electronics(self, names: list[str]) -> None: ...

    def close(self) -> None: ...
