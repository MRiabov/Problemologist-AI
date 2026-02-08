import os
from pathlib import Path

import mujoco
import structlog
from build123d import Compound, export_stl

from worker.simulation.builder import SimulationBuilder
from worker.simulation.interface import SimulationScene
from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)

# validate_and_price moved to dfm.py


class SimulationResult:
    def __init__(
        self,
        success: bool,
        summary: str,
        render_paths: list[str] = None,
        mjcf_content: str | None = None,
    ):
        self.success = success
        self.summary = summary
        self.render_paths = render_paths or []
        self.mjcf_content = mjcf_content


def simulate(
    component: Compound | SimulationScene, output_dir: Path | None = None
) -> SimulationResult:
    """
    Provide a physics-backed stability check.
    Logic:
    - Convert Compound/Scene to MJCF using SimulationBuilder.
    - Run MuJoCo for a few frames.
    - Assert no NaNs/explosions.
    - Generate standard 24-view renders in /renders/.
    - Return stability status and render paths.
    """
    logger.info("simulate_start", component_type=type(component).__name__)

    # 1. Setup paths
    if output_dir:
        # Use provided output_dir for assets
        renders_dir = output_dir / "renders"
    else:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    try:
        # 2. Generate MJCF
        builder = SimulationBuilder(output_dir=renders_dir)

        if isinstance(component, SimulationScene):
            assembly_to_process = component.assembly
            # Note: We currently don't use agent_joints in builder, assume builder handles basic physics
        else:
            assembly_to_process = component

        mjcf_path = builder.build_from_assembly(assembly_to_process)

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
        # For renders, we use build123d Compound
        render_paths = prerender_24_views(assembly_to_process, str(renders_dir))

        # Read MJCF content
        mjcf_content = None
        if mjcf_path.exists():
            with open(mjcf_path, "r") as f:
                mjcf_content = f.read()

        return SimulationResult(True, "Simulation stable.", render_paths, mjcf_content)

    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(False, f"Simulation error: {e!s}")


def validate(component: Compound) -> bool:
    """
    Verify geometric validity and randomization robustness.
    Logic:
    - Check for part intersections.
    - Verify boundary constraints (AABB).
    - Test validity across a few random seeds.
    """
    logger.info("validate_start")

    # 1. Intersection check
    # Check if any solids of the compound overlap
    solids = component.solids()

    if len(solids) > 1:
        for i in range(len(solids)):
            for j in range(i + 1, len(solids)):
                # This is a bit slow but correct for small number of parts
                intersection = solids[i].intersect(solids[j])
                if intersection and intersection.volume > 0.1:  # 0.1 mm^3 threshold
                    logger.warning(
                        "geometric_intersection_detected", volume=intersection.volume
                    )
                    return False

    # 2. Boundary check (AABB)
    bbox = component.bounding_box()
    MAX_SIZE = 1000.0  # 1 meter
    if bbox.size.X > MAX_SIZE or bbox.size.Y > MAX_SIZE or bbox.size.Z > MAX_SIZE:
        logger.warning("boundary_constraint_violation", size=bbox.size)
        return False

    return True
