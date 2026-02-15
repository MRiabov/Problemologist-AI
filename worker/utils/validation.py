import json
import os
from pathlib import Path
from typing import Literal

import structlog
import yaml
from build123d import Compound

from shared.models.schemas import (
    AssemblyDefinition,
    CotsPartEstimate,
    ElectronicsSection,
    FluidDefinition,
    FluidProperties,
    FluidVolume,
    ObjectivesYaml,
)
from shared.simulation.backends import SimulatorBackendType
from worker.simulation.factory import get_simulation_builder
from worker.simulation.loop import FluidMetricResult, SimulationLoop, StressSummary
from worker.workbenches.config import load_config

from .dfm import validate_and_price
from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)


class SimulationResult:
    def __init__(
        self,
        success: bool,
        summary: str,
        render_paths: list[str] | None = None,
        mjcf_content: str | None = None,
        stress_summaries: list[StressSummary] | None = None,
        stress_fields: dict[str, dict] | None = None,
        fluid_metrics: list[FluidMetricResult] | None = None,
        total_cost: float = 0.0,
        total_weight_g: float = 0.0,
        confidence: str = "high",
    ):
        self.success = success
        self.summary = summary
        self.render_paths = render_paths or []
        self.mjcf_content = mjcf_content
        self.stress_summaries = stress_summaries or []
        self.stress_fields = stress_fields or {}
        self.fluid_metrics = fluid_metrics or []
        self.total_cost = total_cost
        self.total_weight_g = total_weight_g
        self.confidence = confidence

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "summary": self.summary,
            "render_paths": self.render_paths,
            "mjcf_content": self.mjcf_content,
            "stress_summaries": [s.model_dump() for s in self.stress_summaries],
            "stress_fields": self.stress_fields,
            "fluid_metrics": [f.model_dump() for f in self.fluid_metrics],
            "total_cost": self.total_cost,
            "total_weight_g": self.total_weight_g,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "SimulationResult":
        return cls(
            success=data["success"],
            summary=data["summary"],
            render_paths=data.get("render_paths", []),
            mjcf_content=data.get("mjcf_content"),
            stress_summaries=[
                StressSummary(**s) for s in data.get("stress_summaries", [])
            ],
            stress_fields=data.get("stress_fields", {}),
            fluid_metrics=[
                FluidMetricResult(**f) for f in data.get("fluid_metrics", [])
            ],
            total_cost=data.get("total_cost", 0.0),
            total_weight_g=data.get("total_weight_g", 0.0),
        )

    def save(self, path: Path):
        path.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> "SimulationResult | None":
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            return cls.from_dict(data)
        except Exception as e:
            logger.warning(
                "failed_to_load_simulation_result", path=str(path), error=str(e)
            )
            return None


LAST_SIMULATION_RESULT: SimulationResult | None = None


def get_stress_report(part_label: str) -> dict | None:
    """Returns the current stress summary for a simulated FEM part."""
    global LAST_SIMULATION_RESULT
    if LAST_SIMULATION_RESULT is None:
        # Try to load from disk
        candidates = [Path("simulation_result.json")]
        if os.getenv("RENDERS_DIR"):
            candidates.append(
                Path(os.getenv("RENDERS_DIR")).parent / "simulation_result.json"
            )

        for p in candidates:
            res = SimulationResult.load(p)
            if res:
                LAST_SIMULATION_RESULT = res
                break

    if LAST_SIMULATION_RESULT is None:
        logger.warning("get_stress_report_called_before_simulation")
        return None

    for summary in LAST_SIMULATION_RESULT.stress_summaries:
        if summary.part_label == part_label:
            data = summary.model_dump()
            # T018: Add diagnostic advice
            sf = summary.safety_factor
            if sf < 1.2:
                data["advice"] = (
                    "CRITICAL: Part is near or at yielding point. Add material or use stronger material."
                )
            elif sf < 1.5:
                data["advice"] = (
                    "WARNING: Safety factor below 1.5 target. Consider reinforcing high-stress areas."
                )
            elif sf > 5.0:
                data["advice"] = (
                    "OPTIMIZATION: Safety factor exceeds 5.0. Consider thinning parts to save cost and weight."
                )
            else:
                data["advice"] = (
                    "SUCCESS: Safety factor is within the ideal 1.5 - 5.0 range."
                )
            return data

    logger.warning("stress_report_part_not_found", part_label=part_label)
    return None


def preview_stress(
    component: Compound,
    view_angles: list[tuple[float, float]] | None = None,
    output_dir: Path | None = None,
) -> list[str]:
    """Renders the component with a von Mises stress heatmap overlay."""
    global LAST_SIMULATION_RESULT
    if LAST_SIMULATION_RESULT is None:
        # Try to load from disk
        candidates = [Path("simulation_result.json")]
        working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
        candidates.append(working_dir / "simulation_result.json")

        for p in candidates:
            res = SimulationResult.load(p)
            if res:
                LAST_SIMULATION_RESULT = res
                break

    if LAST_SIMULATION_RESULT is None:
        logger.warning("preview_stress_called_before_simulation")
        return []

    logger.info(
        "rendering_stress_heatmaps", count=len(LAST_SIMULATION_RESULT.stress_fields)
    )
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    stress_renders_dir = working_dir / "renders" / "stress"
    stress_renders_dir.mkdir(parents=True, exist_ok=True)
    assets_dir = working_dir / "assets"

    from .rendering import render_stress_heatmap
    from shared.simulation.backends import StressField
    import numpy as np

    render_paths = []
    for part_label, field_data in LAST_SIMULATION_RESULT.stress_fields.items():
        field = StressField(
            nodes=np.array(field_data["nodes"]), stress=np.array(field_data["stress"])
        )
        out_path = stress_renders_dir / f"stress_{part_label}.png"

        # T020: Use the exported mesh for better VLM visibility if available
        mesh_path = assets_dir / f"{part_label}.obj"
        if not mesh_path.exists():
            mesh_path = None

        render_stress_heatmap(field, out_path, mesh_path=mesh_path)
        render_paths.append(str(out_path))

    return render_paths


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


def set_soft_mesh(
    part_id: str, enabled: bool = True, output_dir: Path | None = None
) -> bool:
    """Explicitly enables FEM for the scene and marks intent for a specific part."""
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    obj_path = working_dir / "objectives.yaml"

    if obj_path.exists():
        try:
            data = yaml.safe_load(obj_path.read_text())
            objs = ObjectivesYaml(**data)
            objs.physics.fem_enabled = enabled
            if enabled:
                objs.physics.backend = "genesis"  # FEM requires Genesis
            obj_path.write_text(yaml.dump(objs.model_dump()), encoding="utf-8")
            logger.info("set_soft_mesh_enabled", part_id=part_id, fem_enabled=enabled)
            return True
        except Exception as e:
            logger.error("set_soft_mesh_failed", error=str(e))
            return False
    return False


def to_mjcf(component: Compound, renders_dir: Path | None = None) -> str:
    """Convert a build123d Compound to a MuJoCo XML (MJCF) string."""
    if not renders_dir:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    builder = get_simulation_builder(
        output_dir=renders_dir, backend_type=SimulatorBackendType.MUJOCO
    )
    scene_path = builder.build_from_assembly(component)
    return scene_path.read_text()


def calculate_assembly_totals(
    component: Compound,
    electronics: ElectronicsSection | None = None,
    cots_parts: list[CotsPartEstimate] | None = None,
) -> tuple[float, float]:
    """
    Calculate total cost and weight of the assembly including electronics and COTS.
    """
    config = load_config()
    total_cost = 0.0
    total_weight = 0.0

    # 1. Manufactured parts
    children = getattr(component, "children", [])
    if not children:
        children = [component]

    for child in children:
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
        for comp in electronics.components:
            if comp.type == "power_supply" and comp.cots_part_id:
                from shared.cots.parts.electronics import PowerSupply

                try:
                    psu = PowerSupply(size=comp.cots_part_id)
                    total_cost += psu.metadata.get("price", 0.0)
                    total_weight += psu.metadata.get("weight_g", 0.0)
                except Exception:
                    pass
            elif comp.type == "motor" and comp.cots_part_id:
                from shared.cots.parts.motors import ServoMotor

                try:
                    motor = ServoMotor(size=comp.cots_part_id)
                    total_cost += motor.metadata.get("price", 0.0)
                    total_weight += motor.metadata.get("weight_g", 0.0)
                except Exception:
                    pass

        for wire in electronics.wiring:
            length_m = wire.length_mm / 1000.0
            total_cost += length_m * 0.5
            total_weight += length_m * 20.0

    # 3. Generic COTS parts from assembly definition
    if cots_parts:
        for p in cots_parts:
            total_cost += p.unit_cost_usd * p.quantity
            # Weight is not always in CotsPartEstimate, but we can try to find it
            # if we had a more detailed catalog access here.
            # For now, we'll try to use metadata if we can find it in shared catalog.
            try:
                # Heuristic: try to look up weight if not provided
                # In current schema CotsPartEstimate doesn't have weight_g
                # But the indexer extracts it.
                pass
            except Exception:
                pass

    return total_cost, total_weight


def simulate(
    component: Compound,
    output_dir: Path | None = None,
    fem_enabled: bool | None = None,
    particle_budget: int | None = None,
    smoke_test_mode: bool = False,
) -> SimulationResult:
    """Provide a physics-backed stability and objective check."""
    logger.info(
        "simulate_start", fem_enabled=fem_enabled, particle_budget=particle_budget
    )
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    renders_dir = working_dir / "renders"
    renders_dir.mkdir(parents=True, exist_ok=True)

    objectives = None
    assembly_definition = None
    objectives_path = working_dir / "objectives.yaml"
    if objectives_path.exists():
        try:
            data = yaml.safe_load(objectives_path.read_text(encoding="utf-8"))
            objectives = ObjectivesYaml(**data)
        except Exception as e:
            logger.warning("failed_to_load_objectives", error=str(e))

    cost_est_path = working_dir / "assembly_definition.yaml"
    if cost_est_path.exists():
        try:
            data = yaml.safe_load(cost_est_path.read_text(encoding="utf-8"))
            assembly_definition = AssemblyDefinition(**data)
        except Exception as e:
            logger.warning("failed_to_load_assembly_definition", error=str(e))

    backend_type = SimulatorBackendType.MUJOCO
    if objectives and getattr(objectives, "physics", None):
        backend_type = SimulatorBackendType(objectives.physics.backend)

    builder = get_simulation_builder(output_dir=working_dir, backend_type=backend_type)
    moving_parts = assembly_definition.moving_parts if assembly_definition else []
    electronics = assembly_definition.electronics if assembly_definition else None

    scene_path = builder.build_from_assembly(
        component,
        objectives=objectives,
        moving_parts=moving_parts,
        electronics=electronics,
    )

    dynamic_controllers = {}
    control_inputs = {}
    if assembly_definition and assembly_definition.moving_parts:
        try:
            from worker.utils.controllers import sinusoidal

            for part in assembly_definition.moving_parts:
                if part.control:
                    if part.control.mode == "sinusoidal":
                        dynamic_controllers[part.part_name] = (
                            lambda t, p=part.control: (
                                sinusoidal(t, p.speed, p.frequency or 1.0)
                            )
                        )
                    elif part.control.mode == "constant":
                        control_inputs[part.part_name] = part.control.speed
        except Exception as e:
            logger.warning("failed_to_load_controllers", error=str(e))

    # WP2: GPU OOM Retry Logic
    max_retries = 2
    current_particle_budget = particle_budget or (5000 if smoke_test_mode else 100000)
    metrics = None

    for attempt in range(max_retries + 1):
        try:
            loop = SimulationLoop(
                str(scene_path),
                component=component,
                backend_type=backend_type,
                electronics=electronics,
                objectives=objectives,
                smoke_test_mode=smoke_test_mode,
            )
            # Override particle budget if needed (Genesis backend uses it)
            if hasattr(loop, "particle_budget"):
                loop.particle_budget = current_particle_budget

            video_path = renders_dir / "simulation.mp4"
            metrics = loop.step(
                control_inputs=control_inputs,
                duration=30.0,
                dynamic_controllers=dynamic_controllers,
                video_path=video_path,
            )
            break  # Success or normal failure
        except Exception as e:
            error_str = str(e).lower()
            is_oom = (
                "out of memory" in error_str or "cuda_error_out_of_memory" in error_str
            )

            if is_oom and attempt < max_retries:
                old_budget = current_particle_budget
                current_particle_budget = int(current_particle_budget * 0.75)
                logger.warning(
                    "gpu_oom_retry",
                    attempt=attempt + 1,
                    old_budget=old_budget,
                    new_budget=current_particle_budget,
                )

                from shared.observability.events import emit_event
                from shared.observability.schemas import GpuOomRetryEvent

                emit_event(
                    GpuOomRetryEvent(
                        original_particles=old_budget,
                        reduced_particles=current_particle_budget,
                    )
                )
                continue
            else:
                logger.error("simulation_error", error=str(e))
                return SimulationResult(False, f"Simulation error: {e!s}")

    if metrics is None:
        return SimulationResult(False, "Simulation failed to start.")

    try:
        status_msg = metrics.fail_reason or (
            "Goal achieved." if metrics.success else "Simulation stable."
        )

        confidence = metrics.confidence
        if current_particle_budget < (particle_budget or 100000):
            confidence = "approximate"

        render_paths = prerender_24_views(
            component, output_dir=str(renders_dir), backend_type=backend_type
        )
        if video_path.exists():
            render_paths.append(str(video_path))

        mjcf_content = scene_path.read_text() if scene_path.exists() else None

        cost, weight = calculate_assembly_totals(
            component,
            electronics=electronics,
            cots_parts=assembly_definition.cots_parts if assembly_definition else None,
        )

        global LAST_SIMULATION_RESULT
        LAST_SIMULATION_RESULT = SimulationResult(
            metrics.success,
            status_msg,
            render_paths,
            mjcf_content,
            stress_summaries=metrics.stress_summaries,
            stress_fields=metrics.stress_fields,
            fluid_metrics=getattr(metrics, "fluid_metrics", []),
            total_cost=cost,
            total_weight_g=weight,
            confidence=metrics.confidence,
        )

        # T023: Generate stress heatmaps and append to render_paths
        if metrics.stress_fields:
            stress_renders = preview_stress(component, output_dir=working_dir)
            LAST_SIMULATION_RESULT.render_paths.extend(stress_renders)

        try:
            LAST_SIMULATION_RESULT.save(working_dir / "simulation_result.json")
        except Exception as e:
            logger.warning("failed_to_save_simulation_result", error=str(e))

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
                    return (
                        False,
                        f"Geometric intersection detected (volume: {intersection.volume:.2f})",
                    )

    bbox = component.bounding_box()
    if build_zone:
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
            return (
                False,
                f"Build zone violation: bbox {bbox} outside build_zone {build_zone}",
            )
    else:
        if bbox.size.X > 1000.0 or bbox.size.Y > 1000.0 or bbox.size.Z > 1000.0:
            return (
                False,
                f"Boundary constraint violation: size {bbox.size} exceeds 1000.0",
            )

    try:
        renders_dir = str(output_dir / "renders") if output_dir else None

        # Heuristic: use MuJoCo for validation preview unless Genesis is requested
        backend_type = SimulatorBackendType.MUJOCO
        if output_dir:
            obj_path = output_dir / "objectives.yaml"
            if obj_path.exists():
                try:
                    data = yaml.safe_load(obj_path.read_text(encoding="utf-8"))
                    from shared.models.schemas import ObjectivesYaml

                    objs = ObjectivesYaml(**data)
                    if objs.physics and objs.physics.backend:
                        backend_type = SimulatorBackendType(objs.physics.backend)
                except Exception:
                    pass

        prerender_24_views(component, output_dir=renders_dir, backend_type=backend_type)
    except Exception as e:
        logger.warning("validate_render_capture_failed", error=str(e))

    return True, None
