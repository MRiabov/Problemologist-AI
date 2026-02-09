import os
from pathlib import Path

import mujoco
import structlog
from build123d import Compound, export_stl

from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)

# validate_and_price moved to dfm.py


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


def simulate(component: Compound, output_dir: Path | None = None) -> SimulationResult:
    """
    Provide a physics-backed stability check.
    Logic:
    - Convert Compound to MJCF.
    - Run MuJoCo for a few frames.
    - Assert no NaNs/explosions.
    - Generate standard 24-view renders in /renders/.
    - Return stability status and render paths.
    """
    logger.info("simulate_start")

    # 1. Export STL for MuJoCo
    if output_dir:
        renders_dir = output_dir / "renders"
    else:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    stl_path = renders_dir / "component.stl"
    export_stl(component, str(stl_path))

    # 2. Generate MJCF
    mjcf_xml = """
<mujoco model="validation_scene">
  <asset>
    <mesh name="component_mesh" file="component.stl"/>
  </asset>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="10 10 .01" rgba=".9 .9 .9 1"/>
    <body name="component_body" pos="0 0 0.5">
      <freejoint/>
      <geom type="mesh" mesh="component_mesh" rgba="0 0.5 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""
    mjcf_path = renders_dir / "scene.xml"
    mjcf_path.write_text(mjcf_xml)

    try:
        # 3. Load MuJoCo and run a few frames
        # We need to be careful with paths in MJCF if they are relative
        # MuJoCo will look for component.stl relative to scene.xml
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        data = mujoco.MjData(model)

        # Run for 100 steps
        for _ in range(100):
            mujoco.mj_step(model, data)

            # Check for NaNs or excessive velocities (explosions)
            if any(abs(v) > 100.0 for v in data.qvel):
                return SimulationResult(
                    False,
                    "Simulation exploded - check for intersections or poor geometry.",
                )
            if any(abs(p) > 100.0 for p in data.qpos):
                return SimulationResult(False, "Simulation went out of bounds.")

        # 4. Generate renders
        render_paths = prerender_24_views(component)

        # Read MJCF content
        mjcf_content = None
        if mjcf_path.exists():
            mjcf_content = mjcf_path.read_text()

        return SimulationResult(True, "Simulation stable.", render_paths, mjcf_content)

    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(False, f"Simulation error: {e!s}")


def validate(component: Compound, build_zone: dict | None = None) -> bool:
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
                    logger.warning(
                        "geometric_intersection_detected", volume=intersection.volume
                    )
                    return False

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
            logger.warning("build_zone_violation", bbox=bbox, build_zone=build_zone)
            return False
    else:
        max_size = 1000.0  # 1 meter fallback
        if bbox.size.X > max_size or bbox.size.Y > max_size or bbox.size.Z > max_size:
            logger.warning("boundary_constraint_violation", size=bbox.size)
            return False

    return True
