import threading
from typing import Any

import mujoco
import numpy as np

from shared.models.simulation import FluidMetricResult, StressSummary
from shared.enums import FailureReason
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


class MuJoCoBackend(PhysicsBackend):
    _lock = threading.Lock()

    def __init__(self):
        self.model = None
        self.data = None
        self.renderer = None
        self.custom_cameras = {}  # name -> mjvCamera
        self.smoke_test_mode = False

        # ID caches to avoid expensive string lookups in loop
        self._body_id_cache = {}
        self._site_id_cache = {}
        self._actuator_id_cache = {}
        self._tendon_id_cache = {}
        self._camera_id_cache = {}

    def load_scene(self, scene: SimulationScene, render_only: bool = False) -> None:
        with self._lock:
            if scene.scene_path:
                self.model = mujoco.MjModel.from_xml_path(scene.scene_path)
                self.data = mujoco.MjData(self.model)

                # Clear caches on new scene load
                self._body_id_cache.clear()
                self._site_id_cache.clear()
                self._actuator_id_cache.clear()
                self._tendon_id_cache.clear()
                self._camera_id_cache.clear()

                if self.smoke_test_mode:
                    # Accelerate simulation for smoke tests
                    self.model.opt.timestep = 0.05
            else:
                raise ValueError("MuJoCoBackend requires scene_path")

    def _get_body_id(self, name: str) -> int:
        if name not in self._body_id_cache:
            self._body_id_cache[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name
            )
        return self._body_id_cache[name]

    def _get_site_id(self, name: str) -> int:
        if name not in self._site_id_cache:
            self._site_id_cache[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, name
            )
        return self._site_id_cache[name]

    def _get_actuator_id(self, name: str) -> int:
        if name not in self._actuator_id_cache:
            self._actuator_id_cache[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
        return self._actuator_id_cache[name]

    def _get_tendon_id(self, name: str) -> int:
        if name not in self._tendon_id_cache:
            self._tendon_id_cache[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_TENDON, name
            )
        return self._tendon_id_cache[name]

    def _get_camera_id(self, name: str) -> int:
        if name not in self._camera_id_cache:
            self._camera_id_cache[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, name
            )
        return self._camera_id_cache[name]

    def step(self, dt: float) -> StepResult:
        with self._lock:
            if self.model is None or self.data is None:
                raise RuntimeError("Scene not loaded")

        # In MuJoCo, dt is usually model.opt.timestep
        # If passed dt != model.opt.timestep, we might need multiple steps or to change model.opt.timestep
        # For now, we assume dt is a single step or we do multiple steps to reach dt

        target_time = self.data.time + dt
        while self.data.time < target_time:
            # Check for instability before step
            if np.any(np.isnan(self.data.qpos)) or np.any(np.isnan(self.data.qvel)):
                return StepResult(
                    time=self.data.time,
                    success=False,
                    failure_reason=FailureReason.PHYSICS_INSTABILITY,
                )

            mujoco.mj_step(self.model, self.data)

            # Check for instability after step
            if np.any(np.isnan(self.data.qpos)) or np.any(np.isnan(self.data.qvel)):
                return StepResult(
                    time=self.data.time,
                    success=False,
                    failure_reason=FailureReason.PHYSICS_INSTABILITY,
                )

        return StepResult(time=self.data.time, success=True)

    def get_body_state(self, body_id: str) -> BodyState:
        # MuJoCo uses integer IDs or names
        try:
            bid = self._get_body_id(body_id)
        except:
            bid = int(body_id)

        if bid == -1:
            raise ValueError(f"Body {body_id} not found")

        return BodyState(
            pos=tuple(self.data.xpos[bid].tolist()),
            quat=tuple(self.data.xquat[bid].tolist()),
            vel=tuple(self.data.cvel[bid][3:].tolist()),
            angvel=tuple(self.data.cvel[bid][:3].tolist()),
        )

    def get_state(self) -> dict[str, Any]:
        if self.data is None:
            return {}
        return {
            "time": self.data.time,
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "act": self.data.act.tolist() if self.model and self.model.na > 0 else [],
        }

    def get_stress_field(self, body_id: str) -> StressField | None:
        # MuJoCo (rigid only) does not have stress fields
        return None

    def get_max_stress(self) -> float:
        return 0.0

    def get_stress_summaries(self) -> list[StressSummary]:
        return []

    def get_particle_positions(self) -> np.ndarray | None:
        # MuJoCo (rigid only) does not have particles
        return None

    def get_fluid_metrics(self) -> list[FluidMetricResult]:
        return []

    # Rendering & Visualization
    def render(self) -> np.ndarray:
        # Default render (e.g. from first camera or default view)
        cam_name = None
        if self.model and self.model.ncam > 0:
            cam_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 0)

        return self.render_camera(cam_name or "fixed", 640, 480)

    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray:
        # Recreate renderer every time to avoid resolution/framebuffer issues
        if self.renderer is not None:
            try:
                self.renderer.close()
            except:
                pass

        self.renderer = mujoco.Renderer(self.model, width, height)

        # Check if camera exists, otherwise fallback to default view
        cam = self.custom_cameras.get(camera_name, camera_name)
        if isinstance(cam, str):
            cid = self._get_camera_id(cam)
            if cid == -1:
                # logger.warning("mujoco_camera_not_found_falling_back", camera=cam)
                cam = None

        self.renderer.update_scene(self.data, camera=cam)
        return self.renderer.render()

    def set_camera(
        self,
        camera_name: str,
        pos: tuple[float, float, float] | None = None,
        lookat: tuple[float, float, float] | None = None,
        up: tuple[float, float, float] | None = None,
        fov: float | None = None,
    ) -> None:
        if camera_name not in self.custom_cameras:
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            self.custom_cameras[camera_name] = cam

        cam = self.custom_cameras[camera_name]
        if pos is not None:
            # mjvCamera doesn't have direct pos, but lookat and distance/azimuth/elevation
            # For exact pos, we might need a workaround or just use lookat/dist
            # MuJoCo cameras usually look at something.
            # If we want to set pos, we might need to calculate dist/azim/elev
            # For simplicity, we'll support lookat and distance-based approach if pos is used.
            # Or we can use mjv_setCamera (not available in all versions)

            # Simple heuristic: if pos is provided, assume it's for distance calculation if lookat is also there
            if lookat is not None:
                p = np.array(pos)
                l = np.array(lookat)
                diff = p - l
                cam.distance = np.linalg.norm(diff)
                # azim/elev calculation
                cam.azimuth = np.rad2deg(np.arctan2(diff[0], diff[1]))
                cam.elevation = np.rad2deg(np.arcsin(diff[2] / cam.distance))

        if lookat is not None:
            cam.lookat = np.array(lookat)

        if fov is not None:
            # fov is in model.cam_fovy usually, but MjvCamera doesn't have fov
            pass

    def get_camera_matrix(self, camera_name: str) -> np.ndarray:
        # Return 4x4 projection * view matrix or similar
        # This is a bit complex in MuJoCo, for now returning identity or placeholder
        # Actual implementation would use mjv_updateScene and camera params
        return np.eye(4)

    def set_site_pos(self, site_name: str, pos: np.ndarray) -> None:
        sid = self._get_site_id(site_name)
        if sid != -1:
            self.model.site_pos[sid] = pos
        else:
            raise ValueError(f"Site {site_name} not found")

    def get_contact_forces(self) -> list[ContactForce]:
        forces = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            c_force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, c_force)

            # c_force is in contact frame: [normal, tangential1, tangential2, torque1, torque2, torque3]
            # Convert to world frame if needed

            body1_name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                self.model.geom_bodyid[contact.geom1],
            )
            body2_name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                self.model.geom_bodyid[contact.geom2],
            )

            forces.append(
                ContactForce(
                    body1=body1_name or f"body_{self.model.geom_bodyid[contact.geom1]}",
                    body2=body2_name or f"body_{self.model.geom_bodyid[contact.geom2]}",
                    force=tuple(c_force[:3].tolist()),
                    position=tuple(contact.pos.tolist()),
                )
            )
        return forces

    def get_site_state(self, site_name: str) -> SiteState:
        sid = self._get_site_id(site_name)
        if sid == -1:
            raise ValueError(f"Site {site_name} not found")

        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[sid])

        return SiteState(
            pos=tuple(self.data.site_xpos[sid].tolist()),
            quat=tuple(quat.tolist()),
            size=tuple(self.model.site_size[sid].tolist()),
        )

    def get_actuator_state(self, actuator_name: str) -> ActuatorState:
        aid = self._get_actuator_id(actuator_name)
        if aid == -1:
            raise ValueError(f"Actuator {actuator_name} not found")

        # Get actuator velocity. In MuJoCo, this depends on the joint it's attached to.
        vel = 0.0
        # actuator_velocity is available in newer mujoco
        if hasattr(self.data, "actuator_velocity"):
            vel = self.data.actuator_velocity[aid]
        else:
            # Fallback: find joint and use its velocity
            trntype = self.model.actuator_trntype[aid]
            if trntype == mujoco.mjtTrn.mjTRN_JOINT:
                joint_id = self.model.actuator_trnid[aid, 0]
                qvel_adr = self.model.jnt_doveladr[joint_id]
                vel = self.data.qvel[qvel_adr]

        return ActuatorState(
            force=self.data.actuator_force[aid],
            velocity=float(vel),
            ctrl=self.data.ctrl[aid],
            forcerange=tuple(self.model.actuator_forcerange[aid].tolist()),
        )

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        for name, value in control_inputs.items():
            aid = self._get_actuator_id(name)
            if aid != -1:
                self.data.ctrl[aid] = value

    def get_all_body_names(self) -> list[str]:
        return [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            for i in range(self.model.nbody)
        ]

    def get_all_actuator_names(self) -> list[str]:
        return [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(self.model.nu)
        ]

    def get_all_site_names(self) -> list[str]:
        return [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            for i in range(self.model.nsite)
        ]

    def get_all_tendon_names(self) -> list[str]:
        return [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_TENDON, i)
            for i in range(self.model.ntendon)
        ]

    def get_all_camera_names(self) -> list[str]:
        names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            for i in range(self.model.ncam)
        ]
        # Include custom cameras
        for name in self.custom_cameras:
            if name not in names:
                names.append(name)
        return names

    def check_collision(self, body_name: str, site_name: str) -> bool:
        bid = self._get_body_id(body_name)
        sid = self._get_site_id(site_name)

        if bid == -1 or sid == -1:
            return False

        geom_start = self.model.body_geomadr[bid]
        geom_num = self.model.body_geomnum[bid]

        zone_pos = self.data.site_xpos[sid]
        zone_size = self.model.site_size[sid]
        zone_type = self.model.site_type[sid]
        site_mat = self.data.site_xmat[sid].reshape(3, 3)

        # Fallback: check body center if no geoms
        if geom_num == 0:
            body_pos = self.data.xpos[bid]
            diff = body_pos - zone_pos
            v_local = diff @ site_mat
            if zone_type == mujoco.mjtGeom.mjGEOM_BOX:
                return all(np.abs(v_local) <= zone_size[:3])
            if zone_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                return np.linalg.norm(diff) <= zone_size[0]
            return False

        for i in range(geom_num):
            geom_id = geom_start + i
            geom_type = self.model.geom_type[geom_id]
            geom_pos = self.data.geom_xpos[geom_id]
            geom_mat = self.data.geom_xmat[geom_id].reshape(3, 3)

            # Broadphase check: skip if geom is clearly outside zone
            geom_rbound = self.model.geom_rbound[geom_id]
            # Max possible distance from site center to site bounds
            max_site_dist = np.max(zone_size[:3])
            dist = np.linalg.norm(geom_pos - zone_pos)
            if dist > geom_rbound + max_site_dist + 0.01:
                continue

            vertices = []
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[geom_id]
                vert_adr = self.model.mesh_vertadr[mesh_id]
                vert_num = self.model.mesh_vertnum[mesh_id]
                raw_verts = self.model.mesh_vert[vert_adr : vert_adr + vert_num]
                vertices = geom_pos + raw_verts @ geom_mat.T
            else:
                # For non-mesh geoms, just use center for now
                vertices = np.array([geom_pos])

            # Check vertices against the zone
            if zone_type == mujoco.mjtGeom.mjGEOM_BOX:
                diff = vertices - zone_pos
                v_local = diff @ site_mat
                # Use np.atleast_2d to handle single vertex case
                in_box = np.all(np.abs(np.atleast_2d(v_local)) <= zone_size[:3], axis=1)
                if np.any(in_box):
                    return True
            elif zone_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                diff = vertices - zone_pos
                dists = np.linalg.norm(np.atleast_2d(diff), axis=1)
                if np.any(dists < zone_size[0]):
                    return True
        return False

    def get_tendon_tension(self, tendon_name: str) -> float:
        tid = self._get_tendon_id(tendon_name)
        if tid == -1:
            raise ValueError(f"Tendon {tendon_name} not found")
        return float(self.data.ten_force[tid])

    def apply_jitter(self, body_name: str, jitter: tuple[float, float, float]) -> None:
        """Apply position jitter to a body in MuJoCo."""
        body_id = self._get_body_id(body_name)
        if body_id == -1:
            return

        # T019: Locate the free joint of the body to apply jitter instead of hardcoded indices
        jnt_adr = self.model.body_jntadr[body_id]
        jnt_num = self.model.body_jntnum[body_id]

        for i in range(jnt_num):
            j_id = jnt_adr + i
            if self.model.jnt_type[j_id] == mujoco.mjtJoint.mjJNT_FREE:
                q_adr = self.model.jnt_qposadr[j_id]
                self.data.qpos[q_adr : q_adr + 3] += np.array(jitter)
                mujoco.mj_forward(self.model, self.data)
                return

        # If no free joint found, we might be trying to jitter a fixed body or one with specific joints.
        # Fallback to legacy behavior if it's the first body (usually the target)
        if body_id == 1:
            self.data.qpos[0:3] += np.array(jitter)
            mujoco.mj_forward(self.model, self.data)

    def close(self) -> None:
        """Close MuJoCo backend and release resources."""
        if self.renderer:
            try:
                self.renderer.close()
            except Exception:
                pass
            self.renderer = None
        self.data = None
        self.model = None

        self._body_id_cache.clear()
        self._site_id_cache.clear()
        self._actuator_id_cache.clear()
        self._tendon_id_cache.clear()
        self._camera_id_cache.clear()
