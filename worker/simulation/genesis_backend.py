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
        self.entities = {}  # name -> gs.Entity
        self.entity_configs = {}  # name -> dict (from json)
        self.cameras = {}  # name -> gs.Camera
        self.current_time = 0.0
        self.mfg_config = None
        if gs is not None:
            try:
                # Use CPU for tests/CI if GPU not available, but prefer GPU
                gs.init(backend=gs.gpu)
            except Exception as e:
                if "already initialized" in str(e):
                    logger.debug("genesis_already_initialized")
                else:
                    try:
                        gs.init(backend=gs.cpu)
                        logger.info("genesis_init_cpu")
                    except Exception as e2:
                        logger.error("genesis_init_failed", error=str(e2))

    def _load_mfg_config(self):
        if self.mfg_config is None:
            try:
                from worker.workbenches.config import load_config

                self.mfg_config = load_config()
            except Exception as e:
                logger.error("failed_to_load_mfg_config", error=str(e))

    def load_scene(self, scene: SimulationScene) -> None:
        if gs is None:
            raise ImportError("Genesis not installed")

        self.scene_meta = scene
        self._load_mfg_config()

        if "gs_scene" in scene.assets:
            self.scene = scene.assets["gs_scene"]
            return

        # Create new scene if not provided
        self.scene = gs.Scene(show_viewer=False)

        if scene.scene_path and scene.scene_path.endswith(".json"):
            import json
            from pathlib import Path

            scene_dir = Path(scene.scene_path).parent
            try:
                with open(scene.scene_path) as f:
                    data = json.load(f)

                    # 1. Add Entities
                    for ent_cfg in data.get("entities", []):
                        name = ent_cfg["name"]
                        self.entity_configs[name] = ent_cfg

                        if ent_cfg.get("is_zone"):
                            continue

                        material_id = ent_cfg.get("material_id", "aluminum_6061")
                        mat_props = (
                            self.mfg_config.materials.get(material_id)
                            if self.mfg_config
                            else None
                        )

                        # Genesis Material
                        if ent_cfg["type"] == "soft_mesh":
                            # FEM Material
                            material = gs.materials.FEM.Elastic(
                                E=mat_props.youngs_modulus_pa if mat_props else 1e7,
                                nu=mat_props.poissons_ratio if mat_props else 0.45,
                                rho=mat_props.density_kg_m3 if mat_props else 1000,
                            )
                            # Load MSH or OBJ (Genesis can tetrahedralize OBJ)
                            file_path = scene_dir / ent_cfg["file"]
                            entity = self.scene.add_entity(
                                gs.morphs.Mesh(
                                    file=str(file_path),
                                    pos=ent_cfg["pos"],
                                    euler=ent_cfg["euler"],
                                ),
                                material=material,
                            )
                        else:
                            # Rigid Material
                            material = gs.materials.Rigid(
                                rho=mat_props.density_kg_m3 if mat_props else 1000,
                                friction=mat_props.friction_coef if mat_props else 0.5,
                                coup_restitution=(
                                    mat_props.restitution if mat_props else 0.5
                                ),
                            )
                            # Load OBJ
                            obj_path = scene_dir / ent_cfg["file"]
                            entity = self.scene.add_entity(
                                gs.morphs.Mesh(
                                    file=str(obj_path),
                                    pos=ent_cfg["pos"],
                                    euler=ent_cfg["euler"],
                                ),
                                material=material,
                            )

                        self.entities[name] = entity

                    # 2. Add Fluids (MPM)
                    for fluid_cfg in data.get("fluids", []):
                        vol = fluid_cfg["initial_volume"]
                        material = gs.materials.MPM.Liquid(
                            rho=1000,
                        )

                        if vol["type"] == "box":
                            self.scene.add_entity(
                                gs.morphs.Box(
                                    pos=vol["center"],
                                    size=vol["size"],
                                ),
                                material=material,
                            )

            except Exception as e:
                logger.error("failed_to_build_genesis_scene", error=str(e))

        # In Genesis, we must call build() before step()
        if self.scene and not getattr(self.scene, "is_built", False):
            try:
                self.scene.build()
            except Exception as e:
                logger.error("genesis_scene_build_failed", error=str(e))

    def step(self, dt: float) -> StepResult:
        if self.scene is None:
            self.current_time += dt
            return StepResult(time=self.current_time, success=True)

        if not getattr(self.scene, "is_built", False):
            return StepResult(
                time=self.current_time,
                success=False,
                failure_reason="Scene is not built yet.",
            )

        try:
            # Genesis step size is controlled by gs.Scene(sim_options=...)
            # Ideally dt matches what was configured in gs.Scene
            self.scene.step()
        except Exception as e:
            logger.error("genesis_step_failed", error=str(e))
            return StepResult(
                time=self.current_time, success=False, failure_reason=str(e)
            )

        self.current_time += dt
        return StepResult(time=self.current_time, success=True)

    def get_body_state(self, body_id: str) -> BodyState:
        if body_id not in self.entities:
            return BodyState(
                pos=(0, 0, 0), quat=(1, 0, 0, 0), vel=(0, 0, 0), angvel=(0, 0, 0)
            )

        entity = self.entities[body_id]
        state = entity.get_state()

        if hasattr(state, "pos") and state.pos.ndim == 3:
            # FEM or MPM entity: state.pos is [1, n_nodes, 3] or [1, n_particles, 3]
            pos = state.pos[0].mean(axis=0).tolist()
            vel = state.vel[0].mean(axis=0).tolist()
            return BodyState(pos=pos, quat=(1, 0, 0, 0), vel=vel, angvel=(0, 0, 0))
        else:
            # Rigid entity
            return BodyState(
                pos=entity.get_pos().tolist(),
                quat=entity.get_quat().tolist(),
                vel=entity.get_vel().tolist(),
                angvel=entity.get_angvel().tolist(),
            )

    def get_stress_field(self, body_id: str) -> StressField | None:
        if body_id not in self.entities:
            return None

        entity = self.entities[body_id]
        # For FEM, we might want to return per-node stress if available
        # Placeholder for now
        return None

    def get_particle_positions(self) -> np.ndarray | None:
        # For MPM fluids
        all_particles = []
        for name, entity in self.entities.items():
            if "MPM" in str(type(entity)):
                state = entity.get_state()
                if hasattr(state, "pos"):
                    all_particles.append(state.pos[0].cpu().numpy())

        if not all_particles:
            return None

        return np.concatenate(all_particles, axis=0)

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        if not self.scene:
            return np.zeros((height, width, 3), dtype=np.uint8)

        if camera_name not in self.cameras:
            # Add a default camera if not found
            cam = self.scene.add_camera(res=(width, height))
            self.cameras[camera_name] = cam

        cam = self.cameras[camera_name]
        rgb, depth, seg = cam.render()

        if hasattr(rgb, "cpu"):
            rgb = rgb.cpu().numpy()

        return rgb.astype(np.uint8)

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
        return list(self.entities.keys())

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
        if self.scene:
            try:
                # Some versions of Genesis have gs.destroy()
                if hasattr(gs, "destroy"):
                    gs.destroy()
            except Exception:
                pass
            self.scene = None
        self.entities = {}
        self.cameras = {}
