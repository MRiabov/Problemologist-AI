from typing import Any

import numpy as np
import structlog

try:
    import genesis as gs
except ImportError:
    gs = None

from shared.models.simulation import FluidMetricResult, StressSummary
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
        self.cables = []  # wire_id -> dict
        self.ctrl_map = {}  # actuator_name -> float
        self.current_time = 0.0
        self.mfg_config = None
        self.current_particle_multiplier = 1.0
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

        # T017: GPU OOM Auto-Retry Logic
        max_retries = 3
        particle_reduction_factor = 0.75
        self.current_particle_multiplier = 1.0

        for attempt in range(max_retries):
            try:
                self._load_scene_internal(scene)
                # If we get here, it succeeded
                break
            except Exception as e:
                # Check for OOM
                error_str = str(e).lower()
                if (
                    "out of memory" in error_str
                    or "cuda error: out of memory" in error_str
                ):
                    from shared.observability.events import emit_event
                    from shared.observability.schemas import GpuOomRetryEvent

                    original_count = int(10000 * self.current_particle_multiplier)
                    self.current_particle_multiplier *= particle_reduction_factor
                    reduced_count = int(10000 * self.current_particle_multiplier)

                    emit_event(
                        GpuOomRetryEvent(
                            original_particles=original_count,
                            reduced_particles=reduced_count,
                        )
                    )

                    logger.warning(
                        "genesis_oom_detected",
                        attempt=attempt + 1,
                        reduction=particle_reduction_factor,
                    )
                    # Clean up and retry
                    self.close()
                    if attempt == max_retries - 1:
                        logger.error("genesis_oom_persistent")
                        raise
                else:
                    logger.error("failed_to_build_genesis_scene", error=str(e))
                    raise

    def _load_scene_internal(self, scene: SimulationScene) -> None:
        """Internal helper to build the scene, allowing for retries on OOM."""
        if gs is None:
            raise ImportError("Genesis not installed")

        # T014: Particle visualization options from WP06
        self.scene = gs.Scene(
            show_viewer=False,
        )
        self.entities = {}
        self.entity_configs = {}

        if scene.scene_path and (
            scene.scene_path.endswith(".xml") or scene.scene_path.endswith(".mjcf")
        ):
            try:
                # Load MJCF directly
                mjcf_entity = self.scene.add_entity(
                    gs.morphs.MJCF(file=scene.scene_path)
                )
                self.entities["mjcf_scene"] = mjcf_entity
                logger.debug("genesis_mjcf_loaded", path=scene.scene_path)
            except Exception as e:
                logger.error("failed_to_load_mjcf_in_genesis", error=str(e))

        elif scene.scene_path and scene.scene_path.endswith(".json"):
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
                            if mat_props and mat_props.material_class in [
                                "soft",
                                "elastomer",
                            ]:
                                material = gs.materials.FEM.NeoHookean(
                                    E=mat_props.youngs_modulus_pa
                                    if mat_props.youngs_modulus_pa
                                    else 5e6,
                                    nu=mat_props.poissons_ratio
                                    if mat_props.poissons_ratio
                                    else 0.49,
                                    rho=mat_props.density_kg_m3
                                    if mat_props.density_kg_m3
                                    else 1100,
                                )
                            else:
                                material = gs.materials.FEM.Elastic(
                                    E=mat_props.youngs_modulus_pa
                                    if mat_props and mat_props.youngs_modulus_pa
                                    else 68.9e9,
                                    nu=mat_props.poissons_ratio
                                    if mat_props and mat_props.poissons_ratio
                                    else 0.33,
                                    rho=mat_props.density_kg_m3
                                    if mat_props and mat_props.density_kg_m3
                                    else 2700,
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
                                rho=mat_props.density_kg_m3
                                if mat_props and mat_props.density_kg_m3
                                else 2700,
                                friction=mat_props.friction_coef
                                if mat_props and mat_props.friction_coef
                                else 0.5,
                                coup_restitution=(
                                    mat_props.restitution
                                    if mat_props and mat_props.restitution
                                    else 0.5
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

                    # 3. Add Motors / Controls
                    self.motors = data.get("motors", [])
                    self.cables = data.get("cables", [])

                    # T014: Fluid Spawning from FluidDefinition (with WP06 color support)
                    for fluid_cfg in data.get("fluids", []):
                        props = fluid_cfg.get("properties", {})
                        vol = fluid_cfg["initial_volume"]
                        color = fluid_cfg.get("color", [0, 0, 200])
                        # Convert 0-255 to 0-1 and add alpha for transparency
                        color_f = tuple([c / 255.0 for c in color] + [0.8])

                        # MPM Material based on FluidDefinition
                        material = gs.materials.MPM.Liquid(
                            rho=props.get("density_kg_m3", 1000),
                            viscosity=props.get("viscosity_cp", 1.0)
                            * 0.001,  # Convert cP to Pa.s
                        )

                        # Support Box and Sphere spawning volumes
                        # T017: Apply particle multiplier to fidelity
                        n_particles = int(10000 * self.current_particle_multiplier)

                        if vol["type"] == "box":
                            self.scene.add_entity(
                                gs.morphs.Box(
                                    pos=vol["center"],
                                    size=vol.get("size", [0.1, 0.1, 0.1]),
                                    sampler="grid",
                                    n_particles=n_particles,
                                    color=color_f,
                                ),
                                material=material,
                            )
                        elif vol["type"] == "sphere":
                            self.scene.add_entity(
                                gs.morphs.Sphere(
                                    pos=vol["center"],
                                    radius=vol.get("radius", 0.05),
                                    sampler="grid",
                                    n_particles=n_particles,
                                    color=color_f,
                                ),
                                material=material,
                            )

            except Exception as e:
                logger.error("failed_to_parse_genesis_json", error=str(e))
                raise

        # In Genesis, we must call build() before step()
        if self.scene and not getattr(self.scene, "is_built", False):
            self.scene.build()

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

            # T012: Part Breakage Detection
            for name, entity in self.entities.items():
                field = self.get_stress_field(name)
                if field is not None and len(field.stress) > 0:
                    max_stress = np.max(field.stress)

                    # Fetch ultimate stress
                    ent_cfg = self.entity_configs.get(name, {})
                    material_id = ent_cfg.get("material_id", "aluminum_6061")
                    mat_props = (
                        self.mfg_config.materials.get(material_id)
                        if self.mfg_config
                        else None
                    )
                    ultimate_stress = (
                        mat_props.ultimate_stress_pa
                        if mat_props and mat_props.ultimate_stress_pa
                        else 310e6
                    )

                    if max_stress > ultimate_stress:
                        max_idx = np.argmax(field.stress)
                        loc = field.nodes[max_idx].tolist()
                        return StepResult(
                            time=self.current_time,
                            success=False,
                            failure_reason=f"PART_BREAKAGE:{name}",
                        )

            # T017: ELECTRONICS_FLUID_DAMAGE check
            failure = self._check_electronics_fluid_damage()
            if failure:
                return StepResult(
                    time=self.current_time,
                    success=False,
                    failure_reason=failure,
                )

        except Exception as e:
            logger.error("genesis_step_failed", error=str(e))
            return StepResult(
                time=self.current_time, success=False, failure_reason=str(e)
            )

        self.current_time += dt
        return StepResult(time=self.current_time, success=True)

    def _check_electronics_fluid_damage(self) -> str | None:
        """Check if any fluid particles are touching electronic components."""
        # WP3 Forward Compatibility: detect if particles are within bounding boxes
        # of entities marked as electronics in the assembly definition.
        particles = self.get_particle_positions()
        if particles is None or len(particles) == 0:
            return None

        # We need to know which entities are electronics.
        # This information would typically come from the assembly definition.
        # For now, we'll check if the entity has 'is_electronics' in its config.
        for name, entity in self.entities.items():
            cfg = self.entity_configs.get(name, {})
            if cfg.get("is_electronics"):
                # Get bounding box of the entity
                # This is simplified; ideally we use the mesh collision
                state = entity.get_state()
                if hasattr(state, "pos"):
                    # Check distance from each particle to entity center
                    # (Very rough approximation for MVP)
                    center = np.mean(state.pos[0].cpu().numpy(), axis=0)
                    dist = np.linalg.norm(particles - center, axis=1)
                    if np.any(dist < 0.05):  # 5cm threshold
                        logger.info("electronics_fluid_damage", part=name)
                        return f"ELECTRONICS_FLUID_DAMAGE:{name}"
        return None

    def get_body_state(self, body_id: str) -> BodyState:
        logger.debug("genesis_get_body_state_request", body_id=body_id)
        if body_id not in self.entities:
            # Check links within MJCF entities (from WP07)
            for ent in self.entities.values():
                try:
                    # In Genesis, if it's an MJCF entity, it's a RigidEntity
                    # which has a .links attribute
                    target_link = None
                    if hasattr(ent, "links"):
                        for link in ent.links:
                            if link.name == body_id:
                                target_link = link
                                break

                    if target_link:
                        logger.debug("genesis_link_state_found", body_id=body_id)
                        # Use get_pos() etc. if they exist, or fallback to properties
                        pos = (
                            target_link.get_pos().tolist()
                            if hasattr(target_link, "get_pos")
                            else target_link.pos.tolist()
                        )
                        quat = (
                            target_link.get_quat().tolist()
                            if hasattr(target_link, "get_quat")
                            else target_link.quat.tolist()
                        )
                        vel = (
                            target_link.get_vel().tolist()
                            if hasattr(target_link, "get_vel")
                            else target_link.vel.tolist()
                        )

                        angvel = [0, 0, 0]
                        if hasattr(target_link, "get_angvel"):
                            angvel = target_link.get_angvel().tolist()
                        elif hasattr(target_link, "angvel"):
                            angvel = target_link.angvel.tolist()

                        return BodyState(
                            pos=pos,
                            quat=quat,
                            vel=vel,
                            angvel=angvel,
                        )
                except Exception as e:
                    logger.debug(
                        "genesis_get_link_failed", body_id=body_id, error=str(e)
                    )
                    continue

            logger.debug("genesis_body_not_found", body_id=body_id)
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

    def get_state(self) -> dict[str, Any]:
        if self.scene is None:
            return {"time": self.current_time}
        return {
            "time": self.current_time,
            "n_entities": len(self.entities),
        }

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

    def get_stress_summaries(self) -> list[StressSummary]:
        summaries = []
        for name, _ in self.entities.items():
            field = self.get_stress_field(name)
            if field is not None:
                max_stress = np.max(field.stress)
                mean_stress = np.mean(field.stress)
                max_idx = np.argmax(field.stress)

                ent_cfg = self.entity_configs.get(name, {})
                material_id = ent_cfg.get("material_id", "aluminum_6061")
                mat_props = (
                    self.mfg_config.materials.get(material_id)
                    if self.mfg_config
                    else None
                )
                ultimate_stress = (
                    mat_props.ultimate_stress_pa
                    if mat_props and mat_props.ultimate_stress_pa
                    else 310e6
                )
                yield_stress = (
                    mat_props.yield_stress_pa
                    if mat_props and mat_props.yield_stress_pa
                    else 276e6
                )

                summaries.append(
                    StressSummary(
                        part_label=name,
                        max_von_mises_pa=float(max_stress),
                        mean_von_mises_pa=float(mean_stress),
                        safety_factor=ultimate_stress / max_stress
                        if max_stress > 0
                        else 100.0,
                        location_of_max=tuple(field.nodes[max_idx].tolist()),
                        utilization_pct=max_stress / yield_stress * 100.0
                        if yield_stress > 0
                        else 0.0,
                    )
                )
        return summaries

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

    def get_fluid_metrics(self) -> list[FluidMetricResult]:
        return []

    # Rendering & Visualization
    def render(self) -> np.ndarray:
        return self.render_camera("default", 640, 480)

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

    def set_site_pos(self, site_name: str, pos: np.ndarray) -> None:
        # In Genesis, sites are often just coordinate systems.
        # If we have entity configs for them, we can update them there.
        if site_name in self.entity_configs:
            self.entity_configs[site_name]["pos"] = pos.tolist()

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
                        position=tuple(c.pos.cpu().numpy().tolist())
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

    def get_site_state(self, site_name: str) -> SiteState:
        # Query entity_configs for zones (WP3 Forward Compatibility)
        cfg = self.entity_configs.get(site_name)
        if cfg:
            return SiteState(
                pos=tuple(cfg.get("pos", (0, 0, 0))),
                quat=tuple(cfg.get("quat", (1, 0, 0, 0))),
                size=tuple(cfg.get("size", (0.01, 0.01, 0.01))),
            )
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
                ctrl=self.ctrl_map.get(actuator_name, 0.0),
                forcerange=(-1000, 1000),  # Default limit
            )
        except Exception:
            return ActuatorState(force=0.0, velocity=0.0, ctrl=0.0, forcerange=(0, 0))

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        # control_inputs: motor_id -> value
        self.ctrl_map.update(control_inputs)
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
        names = list(self.entities.keys())
        for ent in self.entities.values():
            try:
                # Add link names if it's an articulated entity (from WP07)
                if hasattr(ent, "links"):
                    for link in ent.links:
                        if link.name not in names:
                            names.append(link.name)
            except Exception:
                continue
        return names

    def get_all_actuator_names(self) -> list[str]:
        return [m["part_name"] for m in getattr(self, "motors", [])]

    def get_all_site_names(self) -> list[str]:
        # Return zones from entity_configs (WP3 Forward Compatibility)
        return [name for name, cfg in self.entity_configs.items() if cfg.get("is_zone")]

    def get_all_tendon_names(self) -> list[str]:
        # Return wire IDs from cables list (WP3 Forward Compatibility)
        return [c["wire_id"] for c in getattr(self, "cables", [])]

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

    def get_tendon_tension(self, tendon_name: str) -> float:
        # Genesis doesn't provide direct tendon tension yet,
        # but we might be able to derive it from cable state if implemented.
        # For now, return 0.0 or a placeholder.
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
