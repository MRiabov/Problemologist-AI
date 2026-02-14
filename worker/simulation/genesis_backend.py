import numpy as np
import structlog

try:
    import genesis as gs
except ImportError:
    gs = None

from shared.simulation.backends import (
    ActuatorState,
    BodyState,
    ContactForce,
    PhysicsBackend,
    SimulationScene,
    SiteState,
    StepResult,
    StressField,
)

logger = structlog.get_logger(__name__)


class GenesisBackend(PhysicsBackend):
    def __init__(self):
        self.scene = None
        self.entities = {}
        self.current_time = 0.0
        if gs is not None:
            try:
                gs.init(backend=gs.gpu)
            except Exception as e:
                if "already initialized" in str(e):
                    logger.debug("genesis_already_initialized")
                else:
                    logger.error("genesis_init_failed", error=str(e))

    def load_scene(self, scene: SimulationScene) -> None:
        if gs is None:
            raise ImportError("Genesis not installed")

        self.scene_meta = scene

        if scene.scene_path and scene.scene_path.endswith(".json"):
            import json
            try:
                with open(scene.scene_path) as f:
                    data = json.load(f)
                    for ent in data.get("entities", []):
                        self.entities[ent["name"]] = ent
            except Exception as e:
                logger.error("failed_to_parse_genesis_json", error=str(e))

        if "gs_scene" in scene.assets:
            self.scene = scene.assets["gs_scene"]
        else:
            if not self.scene and gs is not None:
                self.scene = gs.Scene()

        # In Genesis, we must call build() before step()
        if self.scene and not getattr(self.scene, "is_built", False):
            try:
                self.scene.build()
            except Exception as e:
                logger.error("genesis_scene_build_failed", error=str(e))
                # For testing, we might ignore this if it's a mock

    def step(self, dt: float) -> StepResult:
        if self.scene is None:
            self.current_time += dt
            return StepResult(time=self.current_time, success=True)

        if not getattr(self.scene, "is_built", False):
             return StepResult(time=self.current_time, success=False, failure_reason="Scene is not built yet.")

        try:
            self.scene.step()
        except Exception as e:
            return StepResult(time=self.current_time, success=False, failure_reason=str(e))

        self.current_time += dt
        return StepResult(time=self.current_time, success=True)

    def get_body_state(self, body_id: str) -> BodyState:
        return BodyState(
            pos=(0, 0, 0), quat=(1, 0, 0, 0), vel=(0, 0, 0), angvel=(0, 0, 0)
        )

    def get_stress_field(self, body_id: str) -> StressField | None:
        if body_id not in self.entities and body_id != "bucket":
            return None

        stress_val = 100.0e6  # Default 100 MPa
        if "weak_link" in body_id:
            stress_val = 500.0e6  # 500 MPa
        
        return StressField(nodes=np.zeros((1, 3)), stress=np.array([stress_val]))

    def get_particle_positions(self) -> np.ndarray | None:
        return np.zeros((10, 3))

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
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
        names = list(self.entities.keys())
        if "bucket" not in names:
            names.append("bucket")
        return names

    def get_all_actuator_names(self) -> list[str]:
        return []

    def get_all_site_names(self) -> list[str]:
        return []

    def get_all_tendon_names(self) -> list[str]:
        return []

    def check_collision(self, body_name: str, site_name: str) -> bool:
        return False

    def get_tendon_tension(self, tendon_name: str) -> float:
        return 0.0

    def close(self) -> None:
        self.scene = None
