from typing import Any

import mujoco
import numpy as np

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


class MuJoCoBackend(PhysicsBackend):
    def __init__(self):
        self.model = None
        self.data = None
        self.renderer = None
        self.custom_cameras = {}  # name -> mjvCamera

    def load_scene(self, scene: SimulationScene) -> None:
        if scene.scene_path:
            self.model = mujoco.MjModel.from_xml_path(scene.scene_path)
            self.data = mujoco.MjData(self.model)
        else:
            raise ValueError("MuJoCoBackend requires scene_path")

    def step(self, dt: float) -> StepResult:
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
                    failure_reason="instability_detected",
                )

            mujoco.mj_step(self.model, self.data)

            # Check for instability after step
            if np.any(np.isnan(self.data.qpos)) or np.any(np.isnan(self.data.qvel)):
                return StepResult(
                    time=self.data.time,
                    success=False,
                    failure_reason="instability_detected",
                )

        return StepResult(time=self.data.time, success=True, max_stress=0.0)

    def get_body_state(self, body_id: str) -> BodyState:
        # MuJoCo uses integer IDs or names
        try:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
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
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, width, height)

        cam = self.custom_cameras.get(camera_name, camera_name)
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
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
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
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
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
        aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if aid == -1:
            raise ValueError(f"Actuator {actuator_name} not found")

        # Get actuator velocity. In MuJoCo, this depends on the joint it's attached to.
        # Simple heuristic: use data.actuator_velocity if available (it's not in older versions)
        # Or use data.qvel for the joint.

        # For now, let's use 0 or try to find the joint
        vel = 0.0
        # actuator_velocity is available in newer mujoco
        if hasattr(self.data, "actuator_velocity"):
            vel = self.data.actuator_velocity[aid]

        return ActuatorState(
            force=self.data.actuator_force[aid],
            velocity=vel,
            ctrl=self.data.ctrl[aid],
            forcerange=tuple(self.model.actuator_forcerange[aid].tolist()),
        )

    def apply_control(self, control_inputs: dict[str, float]) -> None:
        for name, value in control_inputs.items():
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
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

    def check_collision(self, body_name: str, site_name: str) -> bool:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if bid == -1 or sid == -1:
            return False

        geom_start = self.model.body_geomadr[bid]
        geom_num = self.model.body_geomnum[bid]
        if geom_start == -1:
            return False

        zone_pos = self.data.site_xpos[sid]
        zone_size = self.model.site_size[sid]
        zone_type = self.model.site_type[sid]
        site_mat = self.data.site_xmat[sid].reshape(3, 3)

        for i in range(geom_num):
            geom_id = geom_start + i
            geom_type = self.model.geom_type[geom_id]
            geom_pos = self.data.geom_xpos[geom_id]
            geom_mat = self.data.geom_xmat[geom_id].reshape(3, 3)

            vertices = []
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[geom_id]
                vert_adr = self.model.mesh_vertadr[mesh_id]
                vert_num = self.model.mesh_vertnum[mesh_id]
                raw_verts = self.model.mesh_vert[vert_adr : vert_adr + vert_num]
                vertices = geom_pos + raw_verts @ geom_mat.T
            else:
                vertices = np.array([geom_pos])

            # Check vertices against the zone
            if zone_type == mujoco.mjtGeom.mjGEOM_BOX:
                diff = vertices - zone_pos
                v_local = diff @ site_mat
                in_box = np.all(np.abs(v_local) <= zone_size[:3], axis=1)
                if np.any(in_box):
                    return True
            elif zone_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                dists = np.linalg.norm(vertices - zone_pos, axis=1)
                if np.any(dists < zone_size[0]):
                    return True
        return False

    def get_tendon_tension(self, tendon_name: str) -> float:
        tid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        if tid == -1:
            raise ValueError(f"Tendon {tendon_name} not found")
        return float(self.data.ten_force[tid])

    def close(self) -> None:
        if self.renderer:
            self.renderer.close()
            self.renderer = None
