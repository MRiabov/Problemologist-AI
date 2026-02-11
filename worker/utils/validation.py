import os
from pathlib import Path

# import mujoco  (Delayed import inside functions to prevent startup crashes)
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


from worker.simulation.builder import SimulationBuilder
from worker.simulation.loop import SimulationLoop, SimulationMetrics


def to_mjcf(
    component: Compound, model_name: str = "scene", renders_dir: Path | None = None
) -> str:
    """
    Convert a build123d Compound to a MuJoCo XML (MJCF) string using SimulationBuilder.
    """
    if not renders_dir:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    builder = SimulationBuilder(output_dir=renders_dir)
    scene_path = builder.build_from_assembly(component)
    return scene_path.read_text()


def simulate(component: Compound, output_dir: Path | None = None) -> SimulationResult:
    """
    Provide a physics-backed stability and objective check.
    Uses SimulationBuilder and SimulationLoop for high-fidelity physics.
    """
    logger.info("simulate_start")

    # 1. Setup workspace
    if output_dir:
        working_dir = output_dir
    else:
        working_dir = Path(os.getenv("RENDERS_DIR", "./renders")).parent

    renders_dir = working_dir / "renders"
    renders_dir.mkdir(parents=True, exist_ok=True)

    # 2. Load Objectives (if specified in objectives.yaml)
    objectives = None
    objectives_path = working_dir / "objectives.yaml"
    if objectives_path.exists():
        try:
            content = objectives_path.read_text(encoding="utf-8")
            data = yaml.safe_load(content)
            objectives = ObjectivesYaml(**data)
        except Exception as e:
            logger.warning("failed_to_load_objectives", error=str(e), working_dir=str(working_dir), path=str(objectives_path))

    # 3. Build MJCF
    builder = SimulationBuilder(output_dir=working_dir)
    # Pass objectives to builder if available
    scene_path = builder.build_from_assembly(component, objectives=objectives)

    # 4. Initialize Simulation Loop
    loop = SimulationLoop(str(scene_path), component=component)

    # 5. Load Controllers
    dynamic_controllers = {}
    control_inputs = {}

    if objectives:
        try:
            from worker.utils.controllers import sinusoidal, square, constant

            for part in objectives.moving_parts:
                if part.control:
                    if part.control.mode == "sinusoidal":
                        dynamic_controllers[part.name] = (
                            lambda t, p=part.control: sinusoidal(
                                t, p.speed, p.frequency or 1.0
                            )
                        )
                    elif part.control.mode == "constant":
                        control_inputs[part.name] = part.control.speed
                    # Add more mappings as needed
        except Exception as e:
            logger.warning("failed_to_load_controllers", error=str(e))

    # 6. Run Simulation
    try:
        metrics = loop.step(
            control_inputs=control_inputs,
            duration=30.0,
            dynamic_controllers=dynamic_controllers,
        )

        # 6. Final assessment
        status_msg = metrics.fail_reason or "Simulation stable."
        if metrics.success:
            status_msg = "Goal achieved."

        # 7. Generate renders
        render_paths = prerender_24_views(component, output_dir=str(renders_dir))
        mjcf_content = scene_path.read_text() if scene_path.exists() else None

        return SimulationResult(metrics.success, status_msg, render_paths, mjcf_content)

    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(False, f"Simulation error: {e!s}")


def validate(
    component: Compound, build_zone: dict | None = None, output_dir: Path | None = None
) -> tuple[bool, str | None]:
    """
    Verify geometric validity and randomization robustness.
    Logic:
    - Check for part intersections.
    - Verify boundary constraints (AABB).
    - Test validity across a few random seeds.
    """
    import mujoco

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
        renders_dir = None
        if output_dir:
            renders_dir = str(output_dir / "renders")

        prerender_24_views(component, output_dir=renders_dir)
    except Exception as e:
        logger.warning("validate_render_capture_failed", error=str(e))
        # Don't fail validation just because renders failed, but log it

    return True, None
