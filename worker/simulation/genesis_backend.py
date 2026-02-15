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
        self.motors = []  # part_name -> dict
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
                with Path(scene.scene_path).open() as f:
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

                    # 2. Add Joints (if any)
                    # Note: Genesis often adds joints via the morph or as part of the asset loading
                    # For custom meshes, we might need to set them up.
                    # Currently we assume the backend handles it via entity properties if supported.
                    # Genesis 0.3.x uses links/joints for URDF, but for single meshes we can set dmp properties.

                    # 3. Add Motors / Controls
                    self.motors = data.get("motors", [])
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
        state = entity.get_state()

        # Check if it's an FEM entity
        if hasattr(state, "von_mises"):
            nodes = state.pos[0].cpu().numpy()
            stress = state.von_mises[0].cpu().numpy()
            return StressField(nodes=nodes, stress=stress)

        return None

    def get_particle_positions(self) -> np.ndarray | None:
        # For MPM fluids
        all_particles = []
        for _, entity in self.entities.items():
            # In Genesis, MPM entities have particles
            try:
                state = entity.get_state()
                if hasattr(state, "pos") and not hasattr(state, "von_mises"):
                    # Heuristic: MPM has pos but not von_mises (which FEM has)
                    pos = state.pos[0].cpu().numpy()
                    all_particles.append(pos)
            except Exception:
                continue

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
        rgb, _, _ = cam.render()

        if hasattr(rgb, "cpu"):
            rgb = rgb.cpu().numpy()

        return rgb.astype(np.uint8)

    def set_camera(
        self,
        camera_name: str,
        pos: tuple[float, float, float] | None = None,
        lookat: tuple[float, float, float] | None = None,
        up: tuple[float, float, float] | None = None,
        fov: float | None = None,
    ) -> None:
        if not self.scene:
            return

        if camera_name not in self.cameras:
            # Create with default res, will be updated by render_camera if needed
            cam = self.scene.add_camera(res=(640, 480))
            self.cameras[camera_name] = cam
        
        cam = self.cameras[camera_name]
        
        # Genesis cameras use set_pose or similar
        # Based on Genesis 0.3.x: cam.set_pose(pos=..., lookat=..., up=...)
        kwargs = {}
        if pos is not None:
            kwargs["pos"] = pos
        if lookat is not None:
            kwargs["lookat"] = lookat
        if up is not None:
            kwargs["up"] = up
        
        if kwargs:
            cam.set_pose(**kwargs)
        
        if fov is not None:
            # cam.fov = fov
            pass

    def get_camera_matrix(self, _camera_name: str) -> np.ndarray:
        return np.eye(4)

    def set_site_pos(self, _site_name: str, _pos: np.ndarray) -> None:
        pass

    def get_contact_forces(self) -> list[ContactForce]:
        if not self.scene:
            return []

        # Genesis provides contacts via solver/sim
        # In version 0.3.x, contacts are typically accessed via simulator.get_contacts()
        try:
            contacts = self.scene.sim.get_contacts()
            results = []
            for c in contacts:
                # c has fields like pos, normal, force, entities, etc.
                # c.entities is a tuple of (entity_a, entity_b)
                force = c.force.cpu().numpy() if hasattr(c.force, "cpu") else c.force
                results.append(
                    ContactForce(
                        body1=c.entities[0].name,
                        body2=c.entities[1].name,
                        force=tuple(force.tolist()),
                        pos=tuple(c.pos.cpu().numpy().tolist())
                        if hasattr(c.pos, "cpu")
                        else tuple(c.pos.tolist()),
                        normal=tuple(c.normal.cpu().numpy().tolist())
                        if hasattr(c.normal, "cpu")
                        else tuple(c.normal.tolist()),
                    )
                )
            return results
        except Exception:
            # Fallback if API changed or no contacts
            return []

    def get_site_state(self, _site_name: str) -> SiteState:
        return SiteState(pos=(0, 0, 0), quat=(1, 0, 0, 0), size=(0, 0, 0))

    def get_actuator_state(self, actuator_name: str) -> ActuatorState:
        # Find motor by name in self.motors (which are mapped to entities)
        entity_name = actuator_name
        for motor in getattr(self, "motors", []):
            if motor["part_name"] == actuator_name:
                # In this architecture, we assume entity name matches part_name for moving parts
                break

        if entity_name not in self.entities:
            return ActuatorState(force=0.0, velocity=0.0, ctrl=0.0, forcerange=(0, 0))

        entity = self.entities[entity_name]
        try:
            # Genesis uses DOFs for articulated/controlled entities
            forces = entity.get_dofs_force().cpu().numpy()
            vels = entity.get_dofs_velocity().cpu().numpy()

            force = float(forces[0]) if forces.size > 0 else 0.0
            vel = float(vels[0]) if vels.size > 0 else 0.0

            return ActuatorState(
                force=force,
                velocity=vel,
                ctrl=0.0,
                forcerange=(-1000, 1000),  # Default limit
            )
        except Exception:
            return ActuatorState(force=0.0, velocity=0.0, ctrl=0.0, forcerange=(0, 0))

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        # control_inputs: motor_id -> value
        for motor in getattr(self, "motors", []):
            motor_id = motor["part_name"]
            if motor_id in control_inputs:
                val = control_inputs[motor_id]
                # Map motor to entity
                if motor_id in self.entities:
                    entity = self.entities[motor_id]
                    try:
                        entity.set_dofs_force(np.array([val], dtype=np.float32))
                    except Exception as e:
                        logger.debug(
                            "genesis_apply_control_failed", name=motor_id, error=str(e)
                        )

    def get_all_body_names(self) -> list[str]:
        return list(self.entities.keys())

    def get_all_actuator_names(self) -> list[str]:
        return [m["part_name"] for m in getattr(self, "motors", [])]

    def get_all_site_names(self) -> list[str]:
        return []

    def get_all_tendon_names(self) -> list[str]:
        return []

    def check_collision(self, body_name: str, site_name: str) -> bool:
        """Checks if a body is in collision with another body or site (zone)."""
        # In Genesis, sites are often just entities or zones.
        # If site_name is an entity, check contact.
        target_entity = self.entities.get(body_name)
        site_entity = self.entities.get(site_name)

        if not target_entity or not site_entity:
            # Fallback for zones that are not entities
            # We can check bounding boxes if available in scene meta
            if self.scene_meta:
                for ent_cfg in self.scene_meta.assets.get("entities", []):
                    if ent_cfg.get("name") == site_name and ent_cfg.get("is_zone"):
                        # Check if target body pos is within zone
                        state = self.get_body_state(body_name)
                        pos = state.pos
                        z_min = ent_cfg.get("min", [-1e9, -1e9, -1e9])
                        z_max = ent_cfg.get("max", [1e9, 1e9, 1e9])
                        return all(z_min[i] <= pos[i] <= z_max[i] for i in range(3))

            return False

        # If both are entities, check simulation contacts
        contacts = self.get_contact_forces()
        for c in contacts:
            if (c.body1 == body_name and c.body2 == site_name) or (
                c.body1 == site_name and c.body2 == body_name
            ):
                return True

        return False

    def get_tendon_tension(self, _tendon_name: str) -> float:
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
