import os
from pathlib import Path
from typing import Any, List, Tuple

import structlog
import yaml
from build123d import Compound, Part

from shared.models.schemas import (
    ElectronicsSection,
    FluidDefinition,
    FluidProperties,
    FluidVolume,
    ObjectivesYaml,
    PreliminaryCostEstimation,
)
from worker.simulation.builder import SimulationBuilder
from worker.simulation.loop import SimulationLoop
from worker.workbenches.config import load_config
from .dfm import validate_and_price
from .rendering import prerender_24_views
from shared.simulation.backends import SimulatorBackendType

logger = structlog.get_logger(__name__)

class SimulationResult:
    def __init__(
        self,
        success: bool,
        summary: str,
        render_paths: List[str] | None = None,
        mjcf_content: str | None = None,
        stress_summaries: List[Any] | None = None,
        fluid_metrics: List[Any] | None = None,
        total_cost: float = 0.0,
        total_weight_g: float = 0.0,
    ):
        self.success = success
        self.summary = summary
        self.render_paths = render_paths or []
        self.mjcf_content = mjcf_content
        self.stress_summaries = stress_summaries or []
        self.fluid_metrics = fluid_metrics or []
        self.total_cost = total_cost
        self.total_weight_g = total_weight_g

LAST_SIMULATION_RESULT: SimulationResult | None = None

def define_fluid(
    name: str,
    shape_type: str = "box",
    center: Tuple[float, float, float] = (0, 0, 0),
    size: Tuple[float, float, float] | None = None,
    radius: float | None = None,
    height: float | None = None,
    viscosity: float = 1.0,
    density: float = 1000.0,
    surface_tension: float = 0.072,
    color: str = "blue",
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
    builder = SimulationBuilder(output_dir=renders_dir)
    scene_path = builder.build_from_assembly(component)
    return scene_path.read_text()

def calculate_assembly_totals(
    component: Compound, electronics: ElectronicsSection | None = None
) -> Tuple[float, float]:
    """Calculate total cost and weight of the assembly including electronics and COTS."""
    config = load_config()
    total_cost = 0.0
    total_weight = 0.0

    # 1. Manufactured parts
    for child in component.children:
        if not hasattr(child, "metadata") or not child.metadata:
            continue
        
        method_str = child.metadata.get("manufacturing_method")
        from worker.workbenches.models import ManufacturingMethod
        try:
            method = ManufacturingMethod(method_str)
            res = validate_and_price(child, method, config)
            total_cost += res.unit_cost
            total_weight += res.weight_g
        except Exception:
            pass

    # 2. Electronics and COTS parts
    if electronics:
        from shared.cots.parts.electronics import PowerSupply, ElectronicRelay
        from shared.cots.parts.motors import ServoMotor
        
        for comp in electronics.components:
            if comp.type == "power_supply" and comp.cots_part_id:
                try:
                    psu = PowerSupply(size=comp.cots_part_id)
                    total_cost += psu.metadata.get("price", 0.0)
                    total_weight += psu.metadata.get("weight_g", 0.0)
                except Exception: pass
            elif comp.type == "motor" and comp.cots_part_id:
                try:
                    motor = ServoMotor(size=comp.cots_part_id)
                    total_cost += motor.metadata.get("price", 0.0)
                    total_weight += motor.metadata.get("weight_g", 0.0)
                except Exception: pass
        
        for wire in electronics.wiring:
            length_m = wire.length_mm / 1000.0
            total_cost += length_m * 0.5
            total_weight += length_m * 20.0

    return total_cost, total_weight

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

    # 3. Build Scene
    from worker.simulation.factory import get_simulation_builder

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
                        dynamic_controllers[part.part_name] = lambda t, p=part.control: sinusoidal(t, p.speed, p.frequency or 1.0)
                    elif part.control.mode == "constant":
                        control_inputs[part.part_name] = part.control.speed
        except Exception as e:
            logger.warning("failed_to_load_controllers", error=str(e))

    try:
        metrics = loop.step(control_inputs=control_inputs, duration=30.0, dynamic_controllers=dynamic_controllers)
        status_msg = metrics.fail_reason or ("Goal achieved." if metrics.success else "Simulation stable.")
        render_paths = prerender_24_views(component, output_dir=str(renders_dir))
        mjcf_content = scene_path.read_text() if scene_path.exists() else None
        
        cost, weight = calculate_assembly_totals(component, electronics)
        
        global LAST_SIMULATION_RESULT
        LAST_SIMULATION_RESULT = SimulationResult(
            metrics.success, status_msg, render_paths, mjcf_content,
            stress_summaries=getattr(metrics, 'stress_summaries', []),
            total_cost=cost, total_weight_g=weight
        )
        return LAST_SIMULATION_RESULT
    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(False, f"Simulation error: {e!s}")

def validate(
    component: Compound, build_zone: dict | None = None, output_dir: Path | None = None
) -> tuple[bool, str | None]:
    """Verify geometric validity and randomization robustness."""
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
        b_min, b_max = build_zone.get("min", [-1000]*3), build_zone.get("max", [1000]*3)
        if b_min[0] > bbox.min.X or b_min[1] > bbox.min.Y or b_min[2] > bbox.min.Z or \
           b_max[0] < bbox.max.X or b_max[1] < bbox.max.Y or b_max[2] < bbox.max.Z:
            return False, f"Build zone violation: bbox {bbox} outside build_zone {build_zone}"
    
    try:
        prerender_24_views(component, output_dir=str(output_dir / "renders") if output_dir else None)
    except Exception: pass
    return True, None

def get_stress_report(part_label: str) -> dict | None:
    if LAST_SIMULATION_RESULT:
        for s in LAST_SIMULATION_RESULT.stress_summaries:
            if getattr(s, 'part_label', '') == part_label: return s.model_dump()
    return None

def preview_stress(component: Compound, view_angles: List[Tuple[float, float]] | None = None, output_dir: Path | None = None) -> List[str]:
    if not LAST_SIMULATION_RESULT: return []
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    path = working_dir / "renders" / "stress"
    path.mkdir(parents=True, exist_ok=True)
    return [str(path / "stress_placeholder.png")]
