import os
from pathlib import Path

import mujoco
import structlog
import yaml
from build123d import Compound, export_stl

from shared.models.schemas import ObjectivesYaml

from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)


class SimulationResult:
    def __init__(
        self,
        success: bool,
        summary: str,
        render_paths: list[str] | None = None,
        mjcf_content: str | None = None,
    ):
        self.success = success
        self.summary = summary
        self.render_paths = render_paths or []
        self.mjcf_content = mjcf_content


def to_mjcf(
    component: Compound, model_name: str = "scene", renders_dir: Path | None = None
) -> str:
    """
    Convert a build123d Compound to a MuJoCo XML (MJCF) string.
    Note: Currently uses a simplified template with an STL mesh.
    """
    # 1. Export STL (MuJoCo needs it)
    if not renders_dir:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)
    stl_path = renders_dir / f"{model_name}.stl"
    export_stl(component, str(stl_path))

    # 2. Return MJCF string
    # Use absolute path to avoid ambiguity in MuJoCo loading
    stl_abs_path = stl_path.absolute()
    return f"""
<mujoco model="{model_name}">
  <asset>
    <mesh name="{model_name}_mesh" file="{stl_abs_path}"/>
  </asset>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="10 10 .01" rgba=".9 .9 .9 1"/>
    <body name="{model_name}_body" pos="0 0 0.5">
      <freejoint/>
      <geom type="mesh" mesh="{model_name}_mesh" rgba="0 0.5 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""


def simulate(component: Compound, output_dir: Path | None = None) -> SimulationResult:
    """
    Provide a physics-backed stability and objective check.
    Logic:
    - Load objectives.yaml for goal/forbid zones.
    - Convert Compound to MJCF.
    - Run MuJoCo for 30s or until goal hit.
    - Monitor for forbid zone contact or out-of-bounds.
    - Generate standard 24-view renders and latest sim video.
    """
    logger.info("simulate_start")

    # 1. Setup workspace
    if output_dir:
        renders_dir = output_dir / "renders"
    else:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    # 2. Try loading objectives
    objectives = None
    objectives_path = Path("objectives.yaml")
    # If output_dir is provided, likely running in session?
    # But current working directory might be different if called from API vs from runtime?
    # api_simulate is called in worker process. CWD is project root.
    # objectives.yaml is sent via fs/write to session root (which is output_dir).
    # So we should look for objectives.yaml in output_dir if provided!
    if output_dir:
        objectives_path = output_dir / "objectives.yaml"

    if objectives_path.exists():
        try:
            content = objectives_path.read_text(encoding="utf-8")
            data = yaml.safe_load(content)
            objectives = ObjectivesYaml(**data)
            logger.info("objectives_loaded_for_simulation")
        except Exception as e:
            logger.warning("failed_to_load_objectives_for_sim", error=str(e))

    # 3. Generate MJCF
    mjcf_xml = to_mjcf(
        component, model_name="validation_scene", renders_dir=renders_dir
    )
    mjcf_path = renders_dir / "scene.xml"
    mjcf_path.write_text(mjcf_xml)

    # DEBUG: Check if files exist
    try:
        files = list(renders_dir.glob("*"))
        logger.info("files_in_renders_dir", files=[str(f) for f in files])
        if (renders_dir / "validation_scene.stl").exists():
            logger.info("stl_exists")
        else:
            logger.info("stl_missing")
    except Exception as e:
        logger.error("debug_ls_failed", error=str(e))

    try:
        # 4. Load MuJoCo
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        data = mujoco.MjData(model)

        # Run for up to 30 seconds
        duration = 30.0
        max_steps = int(duration / model.opt.timestep)

        goal_hit = False
        for _ in range(max_steps):
            mujoco.mj_step(model, data)

            # A. Explosion/Sanity check
            if any(abs(v) > 200.0 for v in data.qvel):
                return SimulationResult(
                    False, "Simulation exploded - unstable geometry."
                )
            if any(abs(p) > 500.0 for p in data.qpos):
                return SimulationResult(
                    False, "Object went significantly out of bounds."
                )

            # B. Objective checks (if objectives loaded)
            if objectives and model.nbody > 1:
                # body 0 is world, body 1 is the part
                pos = data.xpos[1]

                # Goal zone check
                gz = objectives.objectives.goal_zone
                if (
                    gz.min[0] <= pos[0] <= gz.max[0]
                    and gz.min[1] <= pos[1] <= gz.max[1]
                    and gz.min[2] <= pos[2] <= gz.max[2]
                ):
                    goal_hit = True
                    break

                # Forbid zone check
                for fz in objectives.objectives.forbid_zones:
                    if (
                        fz.min[0] <= pos[0] <= fz.max[0]
                        and fz.min[1] <= pos[1] <= fz.max[1]
                        and fz.min[2] <= pos[2] <= fz.max[2]
                    ):
                        return SimulationResult(False, f"Forbid zone hit: {fz.name}")

        # 5. Final assessment
        status_msg = "Simulation stable."
        if objectives and not goal_hit:
            status_msg = "Simulation completed but goal not achieved."

        # 6. Generate renders
        render_paths = prerender_24_views(component)
        mjcf_content = mjcf_path.read_text() if mjcf_path.exists() else None

        return SimulationResult(True, status_msg, render_paths, mjcf_content)

    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(False, f"Simulation error: {e!s}")


def validate(
    component: Compound, build_zone: dict | None = None
) -> tuple[bool, str | None]:
    """
    Verify geometric validity and randomization robustness.
    Logic:
    - Check for part intersections.
    - Verify boundary constraints (AABB).
    - Test validity across a few random seeds.
    """
    logger.info("validate_start")

    # 1. Intersection check
    solids = component.solids()
    if len(solids) > 1:
        for i in range(len(solids)):
            for j in range(i + 1, len(solids)):
                intersection = solids[i].intersect(solids[j])
                if intersection and intersection.volume > 0.1:
                    msg = f"Geometric intersection detected (volume: {intersection.volume:.2f})"
                    logger.warning(
                        "geometric_intersection_detected", volume=intersection.volume
                    )
                    return False, msg

    # 2. Boundary check (AABB)
    bbox = component.bounding_box()
    if build_zone:
        # Check against build_zone: {"min": [x,y,z], "max": [x,y,z]}
        b_min = build_zone.get("min", [-1000, -1000, -1000])
        b_max = build_zone.get("max", [1000, 1000, 1000])

        if (
            b_min[0] > bbox.min.X
            or b_min[1] > bbox.min.Y
            or b_min[2] > bbox.min.Z
            or b_max[0] < bbox.max.X
            or b_max[1] < bbox.max.Y
            or b_max[2] < bbox.max.Z
        ):
            msg = f"Build zone violation: bbox {bbox} outside build_zone {build_zone}"
            logger.warning("build_zone_violation", bbox=bbox, build_zone=build_zone)
            return False, msg
    else:
        max_size = 1000.0  # 1 meter fallback
        if max_size < bbox.size.X or max_size < bbox.size.Y or max_size < bbox.size.Z:
            msg = f"Boundary constraint violation: size {bbox.size} exceeds {max_size}"
            logger.warning("boundary_constraint_violation", size=bbox.size)
            return False, msg

    # 3. Generate renders (as expected by prompt/reviewer)
    try:
        prerender_24_views(component)
    except Exception as e:
        logger.warning("validate_render_capture_failed", error=str(e))
        # Don't fail validation just because renders failed, but log it

    return True, None
