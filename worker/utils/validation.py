import os
import yaml
import logging
from pathlib import Path
from typing import Any, List, Tuple, Literal

import structlog
from build123d import Compound, export_stl

from shared.models.schemas import (
    ObjectivesYaml, 
    PreliminaryCostEstimation,
    FluidDefinition,
    FluidProperties,
    FluidVolume,
    ElectronicsRequirements
)
from shared.simulation.backends import SimulatorBackendType
from worker.simulation.factory import get_simulation_builder
from worker.simulation.loop import SimulationLoop, StressSummary

from .rendering import prerender_24_views
from .dfm import validate_and_price
from worker.workbenches.config import load_config

logger = structlog.get_logger(__name__)

class SimulationResult:
    def __init__(
        self,
        success: bool,
        summary: str,
        render_paths: list[str] | None = None,
        mjcf_content: str | None = None,
        stress_summaries: list[StressSummary] | None = None,
        fluid_metrics: list[Any] | None = None,
    ):
        self.success = success
        self.summary = summary
        self.render_paths = render_paths or []
        self.mjcf_content = mjcf_content
        self.stress_summaries = stress_summaries or []
        self.fluid_metrics = fluid_metrics or []

LAST_SIMULATION_RESULT: SimulationResult | None = None

def get_stress_report(part_label: str) -> dict | None:
    """Returns the current stress summary for a simulated FEM part."""
    if LAST_SIMULATION_RESULT is None:
        logger.warning("get_stress_report_called_before_simulation")
        return None

    for summary in LAST_SIMULATION_RESULT.stress_summaries:
        if summary.part_label == part_label:
            return summary.model_dump()

    logger.warning("stress_report_part_not_found", part_label=part_label)
    return None

def preview_stress(
    component: Compound,
    view_angles: list[tuple[float, float]] | None = None,
    output_dir: Path | None = None,
) -> list[str]:
    """Renders the component with a von Mises stress heatmap overlay."""
    if LAST_SIMULATION_RESULT is None:
        logger.warning("preview_stress_called_before_simulation")
        return []

    logger.info("rendering_stress_heatmap_placeholder")
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    stress_renders_dir = working_dir / "renders" / "stress"
    stress_renders_dir.mkdir(parents=True, exist_ok=True)
    return [str(stress_renders_dir / "stress_placeholder.png")]

def define_fluid(
    name: str,
    shape_type: Literal["cylinder", "box", "sphere"],
    center: tuple[float, float, float],
    size: tuple[float, float, float] | None = None,
    radius: float | None = None,
    height: float | None = None,
    viscosity: float = 1.0,
    density: float = 1000,
    surface_tension: float = 0.07,
    color: tuple[int, int, int] = (0, 0, 200),
    output_dir: Path | None = None,
) -> dict:
    """Defines a fluid type for use in the simulation."""
    props = FluidProperties(
        viscosity_cp=viscosity,
        density_kg_m3=density,
        surface_tension_n_m=surface_tension,
    )
    vol = FluidVolume(
        type=shape_type, center=center, size=size, radius=radius, height=height
    )
    fluid = FluidDefinition(
        fluid_id=name, properties=props, initial_volume=vol, color=color
    )

    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    obj_path = working_dir / "objectives.yaml"

    if obj_path.exists():
        data = yaml.safe_load(obj_path.read_text())
        objs = ObjectivesYaml(**data)
        updated = False
        for i, f in enumerate(objs.fluids):
            if f.fluid_id == name:
                objs.fluids[i] = fluid
                updated = True
                break
        if not updated:
            objs.fluids.append(fluid)
        obj_path.write_text(yaml.dump(objs.model_dump()), encoding="utf-8")

    return fluid.model_dump()

def to_mjcf(component: Compound, renders_dir: Path | None = None) -> str:
    """Convert a build123d Compound to a MuJoCo XML (MJCF) string."""
    if not renders_dir:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    builder = get_simulation_builder(output_dir=renders_dir, backend_type=SimulatorBackendType.MUJOCO)
    scene_path = builder.build_from_assembly(component)
    return scene_path.read_text()

def simulate(
    component: Compound,
    output_dir: Path | None = None,
    fem_enabled: bool | None = None,
    particle_budget: int | None = None,
) -> SimulationResult:
    """Provide a physics-backed stability and objective check."""
    logger.info("simulate_start", fem_enabled=fem_enabled, particle_budget=particle_budget)
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    renders_dir = working_dir / "renders"
    renders_dir.mkdir(parents=True, exist_ok=True)

    objectives = None
    cost_estimation = None
    objectives_path = working_dir / "objectives.yaml"
    if objectives_path.exists():
        try:
            data = yaml.safe_load(objectives_path.read_text(encoding="utf-8"))
            objectives = ObjectivesYaml(**data)
        except Exception as e:
            logger.warning("failed_to_load_objectives", error=str(e))

    cost_est_path = working_dir / "preliminary_cost_estimation.yaml"
    if cost_est_path.exists():
        try:
            data = yaml.safe_load(cost_est_path.read_text(encoding="utf-8"))
            cost_estimation = PreliminaryCostEstimation(**data)
        except Exception as e:
            logger.warning("failed_to_load_cost_estimation", error=str(e))

    backend_type = SimulatorBackendType.MUJOCO
    if objectives and getattr(objectives, "physics", None):
        backend_type = SimulatorBackendType(objectives.physics.backend)

    builder = get_simulation_builder(output_dir=working_dir, backend_type=backend_type)
    moving_parts = cost_estimation.moving_parts if cost_estimation else []
    electronics = cost_estimation.electronics if cost_estimation else None

    scene_path = builder.build_from_assembly(
        component,
        objectives=objectives,
        moving_parts=moving_parts,
        electronics=electronics,
    )

    loop = SimulationLoop(
        str(scene_path),
        component=component,
        backend_type=backend_type,
        electronics=electronics,
        objectives=objectives,
    )

    dynamic_controllers = {}
    control_inputs = {}
    if cost_estimation and cost_estimation.moving_parts:
        try:
            from worker.utils.controllers import sinusoidal
            for part in cost_estimation.moving_parts:
                if part.control:
                    if part.control.mode == "sinusoidal":
                        dynamic_controllers[part.part_name] = (
                            lambda t, p=part.control: sinusoidal(t, p.speed, p.frequency or 1.0)
                        )
                    elif part.control.mode == "constant":
                        control_inputs[part.part_name] = part.control.speed
        except Exception as e:
            logger.warning("failed_to_load_controllers", error=str(e))

    try:
        metrics = loop.step(
            control_inputs=control_inputs,
            duration=30.0,
            dynamic_controllers=dynamic_controllers,
        )
        status_msg = metrics.fail_reason or "Simulation stable."
        if metrics.success:
            status_msg = "Goal achieved."

        render_paths = prerender_24_views(component, output_dir=str(renders_dir))
        mjcf_content = scene_path.read_text() if scene_path.exists() else None

        global LAST_SIMULATION_RESULT
        LAST_SIMULATION_RESULT = SimulationResult(
            metrics.success,
            status_msg,
            render_paths,
            mjcf_content,
            stress_summaries=metrics.stress_summaries,
            fluid_metrics=metrics.fluid_metrics
        )
        return LAST_SIMULATION_RESULT
    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(False, f"Simulation error: {e!s}")

def validate(
    component: Compound, build_zone: dict | None = None, output_dir: Path | None = None
) -> tuple[bool, str | None]:
    """Verify geometric validity."""
    logger.info("validate_start")
    solids = component.solids()
    if len(solids) > 1:
        for i in range(len(solids)):
            for j in range(i + 1, len(solids)):
                intersection = solids[i].intersect(solids[j])
                if intersection and intersection.volume > 0.1:
                    return False, f"Geometric intersection detected (volume: {intersection.volume:.2f})"

    bbox = component.bounding_box()
    if build_zone:
        b_min = build_zone.get("min", [-1000, -1000, -1000])
        b_max = build_zone.get("max", [1000, 1000, 1000])
        if (b_min[0] > bbox.min.X or b_min[1] > bbox.min.Y or b_min[2] > bbox.min.Z or
            b_max[0] < bbox.max.X or b_max[1] < bbox.max.Y or b_max[2] < bbox.max.Z):
            return False, f"Build zone violation: bbox {bbox} outside build_zone {build_zone}"
    else:
        if 1000.0 < bbox.size.X or 1000.0 < bbox.size.Y or 1000.0 < bbox.size.Z:
            return False, f"Boundary constraint violation: size {bbox.size} exceeds 1000.0"

    try:
        renders_dir = str(output_dir / "renders") if output_dir else None
        prerender_24_views(component, output_dir=renders_dir)
    except Exception as e:
        logger.warning("validate_render_capture_failed", error=str(e))

    return True, None
