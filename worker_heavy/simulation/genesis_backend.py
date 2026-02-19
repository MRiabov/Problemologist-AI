import threading
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
    _lock = threading.Lock()

    def __init__(self):
        self.scene = None
        self.entities = {}  # name -> gs.Entity
        self.entity_configs = {}  # name -> dict (from json)
        self.cameras = {}  # name -> gs.Camera
        self.motors = []  # part_name -> dict
        self.mjcf_actuators = {}  # name -> {joint: str, force_range: tuple}
        self.cables = {}  # name -> gs.Entity
        self.applied_controls = {}  # name -> float
        self.current_time = 0.0
        self._last_max_stress = 0.0
        self.mfg_config = None
        self.current_particle_multiplier = 1.0
        self.particle_budget = None
        self.smoke_test_mode = False
        self._is_built = False
        self._ensure_initialized()

    def _ensure_initialized(self):
        if gs is not None and not getattr(gs, "_initialized", False):
            import os
            import time

            import torch

            start_t = time.time()

            force_cpu = os.getenv("GENESIS_FORCE_CPU", "0") == "1"
            has_gpu = torch.cuda.is_available() and not force_cpu

            backend = gs.gpu if has_gpu else gs.cpu
            logger.info(
                "genesis_initializing", backend="gpu" if backend == gs.gpu else "cpu"
            )

            max_init_retries = 3
            for attempt in range(max_init_retries):
                try:
                    # Reduce logging in smoke test mode
                    gs.init(
                        backend=backend,
                        logging_level="warning" if self.smoke_test_mode else "info",
                    )
                    logger.info(
                        "genesis_initialized",
                        backend="gpu" if backend == gs.gpu else "cpu",
                        duration=time.time() - start_t,
                    )
                    break
                except Exception as e:
                    if "EGL_BAD_DISPLAY" in str(e) and attempt < max_init_retries - 1:
                        logger.warning("genesis_init_egl_retry", attempt=attempt + 1)
                        time.sleep(1.0)
                        continue

                    if backend == gs.gpu:
                        logger.warning(
                            "genesis_gpu_init_failed_falling_back_to_cpu", error=str(e)
                        )
                        backend = gs.cpu
                        start_t = time.time()
                        try:
                            gs.init(backend=gs.cpu)
                            logger.info(
                                "genesis_initialized",
                                backend="cpu",
                                duration=time.time() - start_t,
                            )
                            break
                        except Exception as e2:
                            logger.error("genesis_init_failed", error=str(e2))
                            raise
                    else:
                        logger.error("genesis_init_failed", error=str(e))
                        raise

    def _load_mfg_config(self):
        if self.mfg_config is None:
            try:
                from worker_heavy.workbenches.config import load_config

                self.mfg_config = load_config()
            except Exception as e:
                logger.error("failed_to_load_mfg_config", error=str(e))

    def load_scene(self, scene: SimulationScene, render_only: bool = False) -> None:
        with self._lock:
            # OPTIMIZATION/FIX: If same scene is already built, skip rebuild.
            # This avoids "Scene is already built" error when multiple calls (simulate + prerender)
            # share the same backend instance.
            if (
                self._is_built
                and self.scene_meta
                and self.scene_meta.scene_path == scene.scene_path
                and self.scene_meta.config == scene.config
            ):
                logger.debug("genesis_backend_reuse_scene", scene_path=scene.scene_path)
                return

            # Ensure fresh state for Genesis global state
            self.close()
            self._ensure_initialized()

            if gs is None:
                raise ImportError("Genesis not installed")

            self.scene_meta = scene
            self._load_mfg_config()

            # Reset state for new scene
            self.entities = {}
            self.entity_configs = {}
            self.cameras = {}
            self.motors = []
            self.mjcf_actuators = {}
            self.cables = {}
            self.applied_controls = {}
            self._is_built = False

            if "gs_scene" in scene.assets:
                self.scene = scene.assets["gs_scene"]
                self._is_built = True
                return

            # T017: GPU OOM Auto-Retry Logic
            max_retries = 3
            particle_reduction_factor = 0.75

            # Initial multiplier based on requested budget (default 100k -> multiplier 1.0)
            requested_budget = getattr(scene.config, "particle_budget", 100000)
            self.current_particle_multiplier = requested_budget / 100000.0

            for attempt in range(max_retries):
                try:
                    self._load_scene_internal(scene, render_only=render_only)
                    # If we get here, it succeeded
                    if not render_only:
                        self._is_built = True
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

    @property
    def is_built(self) -> bool:
        if self.scene is None:
            return False
        # Genesis 0.3.x has .is_built on scene
        if hasattr(self.scene, "is_built"):
            return self.scene.is_built
        return self._is_built

    def _load_scene_internal(
        self, scene: SimulationScene, render_only: bool = False
    ) -> None:
        """Internal helper to build the scene, allowing for retries on OOM."""
        if gs is None:
            raise ImportError("Genesis not installed")

        # Optimization for smoke tests
        is_smoke = getattr(self, "smoke_test_mode", False)
        sim_options = gs.options.SimOptions(
            dt=0.05 if is_smoke else 0.002,
            substeps=1 if is_smoke else 10,
        )

        # T014: Particle visualization options from WP06
        self.scene = gs.Scene(
            sim_options=sim_options,
            show_viewer=False,
            vis_options=gs.options.VisOptions(
                particle_size_scale=1.0,
                render_particle_as="sphere",
            ),
        )

        # Add default ground plane
        self.scene.add_entity(gs.morphs.Plane())

        if scene.scene_path and (
            scene.scene_path.endswith(".xml") or scene.scene_path.endswith(".mjcf")
        ):
            try:
                # WP2: Parse MJCF to find actuators and mapping to joints
                import xml.etree.ElementTree as ET

                tree = ET.parse(scene.scene_path)
                root = tree.getroot()
                for act in root.findall(".//actuator/*"):
                    name = act.get("name")
                    joint_name = act.get("joint")
                    force_range = act.get("forcerange")
                    if name and joint_name:
                        fr = (-1000.0, 1000.0)
                        if force_range:
                            try:
                                parts = [float(x) for x in force_range.split()]
                                if len(parts) == 2:
                                    fr = (parts[0], parts[1])
                            except ValueError:
                                pass
                        self.mjcf_actuators[name] = {
                            "joint": joint_name,
                            "force_range": fr,
                        }

                # Load MJCF directly
                mjcf_entity = self.scene.add_entity(
                    gs.morphs.MJCF(file=scene.scene_path)
                )
                self.entities["mjcf_scene"] = mjcf_entity
                logger.debug(
                    "genesis_mjcf_loaded",
                    path=scene.scene_path,
                    actuators=list(self.mjcf_actuators.keys()),
                )
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
                    self.cables = {}  # Keep as dict, but we'll populate from data

                    # 4. Add Cables (Wiring)
                    for cable_cfg in data.get("cables", []):
                        name = cable_cfg["wire_id"]
                        material = gs.materials.Rigid(rho=8960)  # Copper density

                        cable = self.scene.add_entity(
                            gs.morphs.Cable(
                                points=cable_cfg["points"],
                                radius=cable_cfg["radius"],
                            ),
                            material=material,
                        )
                        self.cables[name] = cable

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
                        # T017: Apply particle multiplier or manual budget override
                        if self.particle_budget:
                            n_particles = self.particle_budget
                        else:
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
        if self.scene and not self._is_built:
            # Add default cameras before building
            try:
                if "main" not in self.cameras:
                    self.cameras["main"] = self.scene.add_camera()
                if "prerender" not in self.cameras:
                    self.cameras["prerender"] = self.scene.add_camera()
            except Exception as e:
                logger.warning("genesis_add_camera_failed_pre_build", error=str(e))

            if render_only:
                # OPTIMIZATION: Genesis can render without building the full physics scene.
                # However, some versions/configurations fail with AttributeError: 'Camera' object has no attribute '_is_batched'
                # if the scene is not built.
                logger.info("genesis_building_scene_for_render_only")

            try:
                logger.info("genesis_building_scene")
                self.scene.build(n_envs=1)
            except Exception as e:
                if "already built" not in str(e).lower():
                    logger.error("genesis_build_failed", error=str(e))
                    raise
            self._is_built = True

    def step(self, dt: float) -> StepResult:
        with self._lock:
            if self.scene is None:
                self.current_time += dt
                return StepResult(time=self.current_time, success=True)

        if not self._is_built:
            # Fallback for unexpected state
            logger.warning("genesis_step_called_unbuilt_attempting_sync")
            if getattr(self.scene, "is_built", False):
                self._is_built = True
            else:
                return StepResult(
                    time=self.current_time,
                    success=False,
                    failure_reason="Scene is not built yet.",
                )

        try:
            # Genesis step size is controlled by gs.Scene(sim_options=...)
            # Ideally dt matches what was configured in gs.Scene
            # Reduce debug logging overhead: only log every 100 steps
            self._step_counter = getattr(self, "_step_counter", 0) + 1
            if self._step_counter % 100 == 0:
                logger.debug(
                    "genesis_scene_stepping",
                    is_built=self._is_built,
                    scene_is_built=getattr(self.scene, "is_built", False),
                )
            self.scene.step()

            # WP2: Ensure current_time matches Genesis physics clock
            actual_dt = dt
            if (
                hasattr(self.scene, "sim_options")
                and self.scene.sim_options is not None
            ):
                actual_dt = getattr(self.scene.sim_options, "dt", dt)

            self.current_time += actual_dt

            # T012: Part Breakage Detection & Global Stress Tracking
            self._last_max_stress = 0.0
            for name in self.entities:
                field = self.get_stress_field(name)
                if field is not None and len(field.stress) > 0:
                    max_stress = np.max(field.stress)
                    self._last_max_stress = max(
                        self._last_max_stress, float(max_stress)
                    )

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
                        # max_idx = np.argmax(field.stress)  # Not used currently
                        # loc = field.nodes[max_idx].tolist()  # Not used currently
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
        try:
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
                            # T001: Genesis returns batched tensors, take [0] and ensure it's on CPU
                            def _to_flat_list(val):
                                if hasattr(val, "cpu"):
                                    val = val.cpu().numpy()
                                if isinstance(val, np.ndarray):
                                    if val.ndim > 1:
                                        return val[0].tolist()
                                    return val.tolist()
                                if (
                                    isinstance(val, list)
                                    and len(val) > 0
                                    and isinstance(val[0], list)
                                ):
                                    return val[0]
                                return val

                            pos = _to_flat_list(
                                target_link.get_pos()
                                if hasattr(target_link, "get_pos")
                                else target_link.pos
                            )
                            quat = _to_flat_list(
                                target_link.get_quat()
                                if hasattr(target_link, "get_quat")
                                else target_link.quat
                            )
                            vel = _to_flat_list(
                                target_link.get_vel()
                                if hasattr(target_link, "get_vel")
                                else target_link.vel
                            )

                            angvel = [0, 0, 0]
                            if hasattr(target_link, "get_angvel"):
                                angvel = _to_flat_list(target_link.get_angvel())
                            elif hasattr(target_link, "angvel"):
                                angvel = _to_flat_list(target_link.angvel)

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
                    pos=[0.0, 0.0, 0.0],
                    quat=[1.0, 0.0, 0.0, 0.0],
                    vel=[0.0, 0.0, 0.0],
                    angvel=[0.0, 0.0, 0.0],
                )

            entity = self.entities[body_id]
            logger.debug("genesis_calling_get_state", body_id=body_id)
            state = entity.get_state()
            logger.debug("genesis_get_state_returned", body_id=body_id)

            def _to_flat_list(val):
                if hasattr(val, "cpu"):
                    val = val.cpu().numpy()
                if isinstance(val, np.ndarray):
                    if val.ndim > 1:
                        return val[0].tolist()
                    return val.tolist()
                if isinstance(val, list) and len(val) > 0 and isinstance(val[0], list):
                    return val[0]
                return val

            if hasattr(state, "pos") and state.pos.ndim == 3:
                # FEM or MPM entity: state.pos is [1, n_nodes, 3] or [1, n_particles, 3]
                pos = state.pos[0].cpu().numpy().mean(axis=0).tolist()
                vel = state.vel[0].cpu().numpy().mean(axis=0).tolist()
                return BodyState(pos=pos, quat=(1, 0, 0, 0), vel=vel, angvel=(0, 0, 0))
            return BodyState(
                pos=_to_flat_list(entity.get_pos()),
                quat=_to_flat_list(entity.get_quat()),
                vel=_to_flat_list(entity.get_vel()),
                angvel=_to_flat_list(entity.get_ang()),
            )
        except BaseException as e:
            import traceback

            logger.error(
                "genesis_get_body_state_error_base",
                error=str(e),
                type=type(e).__name__,
                stack="".join(traceback.format_stack()),
            )
            raise

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

    def get_max_stress(self) -> float:
        return self._last_max_stress

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
        with self._lock:
            if not self.scene:
                return np.zeros((height, width, 3), dtype=np.uint8)

        if camera_name not in self.cameras:
            # Add a default camera if not found
            cam = self.scene.add_camera(res=(width, height))
            self.cameras[camera_name] = cam

        cam = self.cameras[camera_name]
        # Genesis can return (rgb, depth, segmentation) or more
        res = cam.render()
        if isinstance(res, tuple):
            rgb = res[0]
        else:
            rgb = res

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
        # logger.debug("genesis_get_contact_forces_start")
        if not self.scene:
            return []

        # Genesis provides contacts via solver/sim
        # In version 0.3.x, contacts are typically accessed via simulator.get_contacts()
        try:
            # logger.debug("genesis_calling_sim_get_contacts")
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
                    )
                )
            return results
        except Exception as e:
            import traceback

            logger.warning(
                "genesis_get_contact_forces_failed",
                error=str(e),
                stack="".join(traceback.format_stack()),
            )
            # Fallback if API changed or no contacts
            return []

    def get_site_state(self, _site_name: str) -> SiteState:
        return SiteState(pos=(0, 0, 0), quat=(1, 0, 0, 0), size=(0, 0, 0))

    def get_actuator_state(self, actuator_name: str) -> ActuatorState:
        ctrl_val = self.applied_controls.get(actuator_name, 0.0)

        # 1. Check MJCF actuators
        if actuator_name in self.mjcf_actuators:
            info = self.mjcf_actuators[actuator_name]
            joint_name = info["joint"]
            entity = self.entities.get("mjcf_scene")
            if entity:
                try:
                    # Find joint by name
                    joint = None
                    for j in entity.joints:
                        if j.name == joint_name:
                            joint = j
                            break

                    if joint:
                        idx = joint.dof_start
                        forces = entity.get_dofs_force().cpu().numpy()
                        vels = entity.get_dofs_velocity().cpu().numpy()

                        return ActuatorState(
                            force=float(forces[idx]),
                            velocity=float(vels[idx]),
                            ctrl=ctrl_val,
                            forcerange=info["force_range"],
                        )
                except Exception as e:
                    logger.debug("failed_to_get_mjcf_actuator_state", error=str(e))

        # 2. Check standard motors
        entity_name = actuator_name
        for motor in getattr(self, "motors", []):
            if motor["part_name"] == actuator_name:
                break

        if entity_name in self.entities:
            entity = self.entities[entity_name]
            try:
                forces = entity.get_dofs_force().cpu().numpy()
                vels = entity.get_dofs_velocity().cpu().numpy()

                force = float(forces[0]) if forces.size > 0 else 0.0
                vel = float(vels[0]) if vels.size > 0 else 0.0

                return ActuatorState(
                    force=force,
                    velocity=vel,
                    ctrl=ctrl_val,
                    forcerange=(-1000, 1000),
                )
            except Exception:
                pass

        return ActuatorState(force=0.0, velocity=0.0, ctrl=ctrl_val, forcerange=(0, 0))

    def _set_entity_dofs_force(self, entity, forces):
        """Helper to set DOFs force, supporting different Genesis versions."""
        if hasattr(entity, "control_dofs_force"):
            entity.control_dofs_force(forces)
        elif hasattr(entity, "set_dofs_force"):
            entity.set_dofs_force(forces)
        else:
            logger.warning("entity_missing_force_control_method", entity=str(entity))

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        # control_inputs: motor_id -> value
        for motor_id, val in control_inputs.items():
            self.applied_controls[motor_id] = val

            # 1. Handle MJCF actuators
            if motor_id in self.mjcf_actuators:
                info = self.mjcf_actuators[motor_id]
                joint_name = info["joint"]
                entity = self.entities.get("mjcf_scene")
                if entity:
                    try:
                        # Find joint by name
                        joint = None
                        for j in entity.joints:
                            if j.name == joint_name:
                                joint = j
                                break

                        if joint:
                            idx = joint.dof_start
                            # We need to set all DOFs or use specific indices
                            forces = entity.get_dofs_force()
                            forces[idx] = val
                            self._set_entity_dofs_force(entity, forces)
                    except Exception as e:
                        logger.debug("mjcf_apply_control_failed", error=str(e))
                continue

            # 2. Handle standard motors
            if motor_id in self.entities:
                entity = self.entities[motor_id]
                try:
                    self._set_entity_dofs_force(
                        entity, np.array([val], dtype=np.float32)
                    )
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
        names = [m["part_name"] for m in getattr(self, "motors", [])]
        names.extend(list(self.mjcf_actuators.keys()))
        return names

    def get_all_site_names(self) -> list[str]:
        return [name for name, cfg in self.entity_configs.items() if cfg.get("is_zone")]

    def get_all_tendon_names(self) -> list[str]:
        return list(self.cables.keys())

    def check_collision(self, body_name: str, site_name: str) -> bool:
        """Checks if a body is in collision with another body or site (zone)."""
        logger.debug("genesis_check_collision_start", body=body_name, site=site_name)
        # In Genesis, sites are often just entities or zones.
        # If site_name is an entity, check contact.
        target_entity = self.entities.get(body_name)
        site_entity = self.entities.get(site_name)

        if not target_entity or not site_entity:
            # Check if it's a zone in entity_configs
            ent_cfg = self.entity_configs.get(site_name)
            if not ent_cfg and self.scene_meta:
                # Fallback for scene_meta
                for cfg in self.scene_meta.assets.get("entities", []):
                    if cfg.get("name") == site_name:
                        ent_cfg = cfg
                        break

            if ent_cfg and ent_cfg.get("is_zone"):
                # Check if target body pos is within zone
                state = self.get_body_state(body_name)
                pos = state.pos

                # Handle both min/max and pos/size representations
                if "min" in ent_cfg and "max" in ent_cfg:
                    z_min = ent_cfg["min"]
                    z_max = ent_cfg["max"]
                elif "pos" in ent_cfg and "size" in ent_cfg:
                    center = ent_cfg["pos"]
                    half_extents = ent_cfg["size"]
                    z_min = [center[i] - half_extents[i] for i in range(3)]
                    z_max = [center[i] + half_extents[i] for i in range(3)]
                else:
                    return False

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
        """Returns the average tension along the cable."""
        if tendon_name not in self.cables:
            raise ValueError(f"Tendon '{tendon_name}' not found in scene")

        cable = self.cables[tendon_name]
        try:
            # Genesis cables (MPM or Rigid) might have stress or force data.
            # For a Rigid cable, we can check forces between nodes.
            # This is an approximation.
            state = cable.get_state()
            if hasattr(state, "force"):
                # Average force magnitude
                forces = state.force.cpu().numpy()
                return float(np.mean(np.linalg.norm(forces, axis=-1)))
            return 0.0
        except Exception:
            return 0.0

    def apply_jitter(self, body_name: str, jitter: tuple[float, float, float]) -> None:
        """Apply position jitter to a body in Genesis using qpos."""
        import torch
        import genesis as gs

        if body_name in self.entities:
            entity = self.entities[body_name]
            qpos = entity.get_qpos()  # Returns a torch tensor [n_dofs]
            # Assuming first 3 are position for free bodies or similar
            # If it has DOFs, jitter the first 3 (which are usually root translations)
            if qpos.shape[0] >= 3:
                jitter_tensor = torch.tensor(
                    jitter, dtype=qpos.dtype, device=qpos.device
                )
                qpos[:3] += jitter_tensor
                entity.set_qpos(qpos)
        else:
            # Check links within entities
            for ent in self.entities.values():
                if hasattr(ent, "links"):
                    for link in ent.links:
                        if link.name == body_name:
                            # link.q_start/q_end give indices into entity qpos
                            qpos = ent.get_qpos()
                            if link.q_start + 3 <= qpos.shape[0]:
                                jitter_tensor = torch.tensor(
                                    jitter, dtype=qpos.dtype, device=qpos.device
                                )
                                qpos[link.q_start : link.q_start + 3] += jitter_tensor
                                ent.set_qpos(qpos)
                            return

    def close(self) -> None:
        try:
            if self.scene and hasattr(self.scene, "destroy"):
                try:
                    self.scene.destroy()
                except Exception as e:
                    logger.debug("scene_destroy_failed", error=str(e))
        except Exception:
            pass
        self.scene = None
        self.entities = {}
        self.cables = {}
        self.cameras = {}
        self._is_built = False
