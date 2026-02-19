import json
import os
from pathlib import Path
from typing import Any, Literal

import structlog
import yaml
from build123d import Compound

from shared.enums import ElectronicComponentType, MotorControlMode
from shared.models.schemas import (
    AssemblyDefinition,
    CotsPartEstimate,
    ElectronicsSection,
    FluidDefinition,
    FluidProperties,
    FluidVolume,
    ObjectivesYaml,
)
from shared.models.simulation import (
    SimulationResult,
)
from shared.simulation.backends import StressField
from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.factory import get_simulation_builder
from worker.workbenches.config import load_config

from .dfm import validate_and_price
from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)


def load_simulation_result(path: Path) -> SimulationResult | None:
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return SimulationResult.model_validate(data)
    except Exception as e:
        logger.warning("failed_to_load_simulation_result", path=str(path), error=str(e))
        return None


def save_simulation_result(result: SimulationResult, path: Path):
    path.write_text(result.model_dump_json(indent=2), encoding="utf-8")


def get_stress_report(part_label: str, output_dir: Path | None = None) -> dict | None:
    """Returns the worst-case stress summary for a simulated FEM part."""
    # Try to load from disk
    candidates = [Path("simulation_result.json")]
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    candidates.append(working_dir / "simulation_result.json")

    res = None
    for p in candidates:
        res = load_simulation_result(p)
        if res:
            break

    if res is None:
        logger.warning("get_stress_report_called_before_simulation")
        return None

    worst_summary = None
    min_sf = float("inf")

    for summary in res.stress_summaries:
        if summary.part_label == part_label and summary.safety_factor < min_sf:
            min_sf = summary.safety_factor
            worst_summary = summary

    if worst_summary:
        res_dict = worst_summary.model_dump()
        sf = res_dict.get("safety_factor", 10.0)
        if sf < 1.2:
            res_dict["advice"] = (
                "Safety factor critical (below 1.2). Part will likely fail. "
                "Reinforce geometry at the location of maximum stress."
            )
        elif sf < 1.5:
            res_dict["advice"] = (
                "Safety factor low (below 1.5). "
                "Consider adding material to reach target range (1.5 - 5.0)."
            )
        elif sf > 5.0:
            res_dict["advice"] = (
                "Safety factor high (over 5.0). Part might be over-engineered. "
                "Consider removing material to reduce cost and weight."
            )
        else:
            res_dict["advice"] = "Safety factor is within acceptable range (1.5 - 5.0)."
        return res_dict

    logger.warning("stress_report_part_not_found", part_label=part_label)
    return None


def preview_stress(
    _component: Compound,
    _view_angles: list[tuple[float, float]] | None = None,
    output_dir: Path | None = None,
) -> list[str]:
    """Renders the component with a von Mises stress heatmap overlay."""
    # Try to load from disk
    candidates = [Path("simulation_result.json")]
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    candidates.append(working_dir / "simulation_result.json")

    res = None
    for p in candidates:
        res = load_simulation_result(p)
        if res:
            break

    if res is None:
        logger.warning("preview_stress_called_before_simulation")
        return []

    logger.info("rendering_stress_heatmaps", count=len(res.stress_fields))
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    stress_renders_dir = working_dir / "renders" / "stress"
    stress_renders_dir.mkdir(parents=True, exist_ok=True)
    assets_dir = working_dir / "assets"

    import numpy as np

    from .rendering import render_stress_heatmap

    render_paths = []
    for part_label, field_data in res.stress_fields.items():
        # T019: Use attribute access for StressFieldData model (WP2)
        nodes = getattr(field_data, "nodes", None) or field_data["nodes"]
        stress = getattr(field_data, "stress", None) or field_data["stress"]
        field = StressField(nodes=np.array(nodes), stress=np.array(stress))
        out_path = stress_renders_dir / f"stress_{part_label}.png"

        # Use the exported mesh for better VLM visibility if available
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
        obj_path.write_text(yaml.dump(objs.model_dump(mode="json")), encoding="utf-8")
    else:
        logger.warning("define_fluid_objectives_not_found", path=str(obj_path))

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
                # FEM currently requires Genesis backend
                objs.physics.backend = "genesis"
            obj_path.write_text(
                yaml.dump(objs.model_dump(mode="json")), encoding="utf-8"
            )
            logger.info("set_soft_mesh_enabled", part_id=part_id, fem_enabled=enabled)
            return True
        except Exception as e:
            logger.error("set_soft_mesh_failed", error=str(e))
            return False
    return False


def to_mjcf(
    component: Compound, renders_dir: Path | None = None, smoke_test_mode: bool = False
) -> str:
    """Convert a build123d Compound to a MuJoCo XML (MJCF) string."""
    if not renders_dir:
        renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    renders_dir.mkdir(parents=True, exist_ok=True)

    builder = get_simulation_builder(
        output_dir=renders_dir, backend_type=SimulatorBackendType.GENESIS
    )
    scene_path = builder.build_from_assembly(component, smoke_test_mode=smoke_test_mode)
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
        metadata = getattr(child, "metadata", None)
        if not metadata:
            continue

        method = getattr(metadata, "manufacturing_method", None)
        from worker.workbenches.models import ManufacturingMethod

        try:
            if isinstance(method, str):
                method = ManufacturingMethod(method)

            if not method:
                continue

            res = validate_and_price(child, method, config)
            total_cost += res.unit_cost
            total_weight += res.weight_g
        except Exception as e:
            logger.error(
                "failed_to_price_manufactured_part",
                part=getattr(child, "label", "unknown"),
                error=str(e),
            )

    # 2. Electronics and COTS parts
    if electronics:
        for comp in electronics.components:
            if comp.type == ElectronicComponentType.POWER_SUPPLY and comp.cots_part_id:
                from shared.cots.parts.electronics import PowerSupply

                try:
                    psu = PowerSupply(size=comp.cots_part_id)
                    total_cost += getattr(psu, "price", 0.0)
                    total_weight += getattr(psu, "weight_g", 0.0)
                except Exception as e:
                    logger.error(
                        "failed_to_price_psu",
                        cots_id=comp.cots_part_id,
                        error=str(e),
                    )
            elif comp.type == ElectronicComponentType.MOTOR and comp.cots_part_id:
                from shared.cots.parts.motors import ServoMotor

                try:
                    motor = ServoMotor(size=comp.cots_part_id)
                    total_cost += getattr(motor, "price", 0.0)
                    total_weight += getattr(motor, "weight_g", 0.0)
                except Exception as e:
                    logger.error(
                        "failed_to_price_motor",
                        cots_id=comp.cots_part_id,
                        error=str(e),
                    )

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
            import contextlib

            with contextlib.suppress(Exception):
                # Heuristic: try to look up weight if not provided
                # In current schema CotsPartEstimate doesn't have weight_g
                # But the indexer extracts it.
                pass

    return total_cost, total_weight


def simulate_subprocess(
    script_path: str,
    session_root: str,
    script_content: str | None = None,
    output_dir: Path | None = None,
    smoke_test_mode: bool = False,
    backend: Any | None = None,
    session_id: str | None = None,
    particle_budget: int | None = None,
) -> SimulationResult:
    """Serializable entry point for ProcessPoolExecutor."""
    # Ensure events are written to the session's event log
    if session_root:
        os.environ["EVENTS_FILE"] = str(Path(session_root) / "events.jsonl")

    from worker.utils.loader import load_component_from_script

    component = load_component_from_script(
        script_path=script_path,
        session_root=session_root,
        script_content=script_content,
    )
    return simulate(
        component=component,
        output_dir=output_dir,
        smoke_test_mode=smoke_test_mode,
        backend=backend,
        session_id=session_id,
        particle_budget=particle_budget,
    )


def simulate(
    component: Compound,
    output_dir: Path | None = None,
    fem_enabled: bool | None = None,
    particle_budget: int | None = None,
    smoke_test_mode: bool = False,
    backend: SimulatorBackendType | None = None,
    session_id: str | None = None,
) -> SimulationResult:
    """Provide a physics-backed stability and objective check."""
    from worker.simulation.loop import SimulationLoop

    logger.info(
        "simulate_start",
        fem_enabled=fem_enabled,
        particle_budget=particle_budget,
        backend=backend,
        session_id=session_id,
    )
    working_dir = output_dir or Path(os.getenv("RENDERS_DIR", "./renders")).parent
    renders_dir = working_dir / "renders"
    renders_dir.mkdir(parents=True, exist_ok=True)

    objectives = None
    assembly_definition = None
    objectives_path = working_dir / "objectives.yaml"
    if objectives_path.exists():
        content = objectives_path.read_text(encoding="utf-8")
        if "[TEMPLATE]" not in content:
            try:
                data = yaml.safe_load(content)
                objectives = ObjectivesYaml(**data)
            except Exception as e:
                logger.error("failed_to_load_objectives", error=str(e))

    cost_est_path = working_dir / "assembly_definition.yaml"
    if cost_est_path.exists():
        try:
            data = yaml.safe_load(cost_est_path.read_text(encoding="utf-8"))
            assembly_definition = AssemblyDefinition(**data)
        except Exception as e:
            logger.error("failed_to_load_assembly_definition", error=str(e))

    backend_type = backend
    if backend_type is None:
        backend_type = SimulatorBackendType.GENESIS
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
        smoke_test_mode=smoke_test_mode,
    )

    from worker.simulation.loop import SimulationLoop

    loop = SimulationLoop(
        str(scene_path),
        component=component,
        backend_type=backend_type,
        electronics=electronics,
        objectives=objectives,
        smoke_test_mode=smoke_test_mode,
        session_id=session_id,
        particle_budget=particle_budget,
    )

    dynamic_controllers = {}
    control_inputs = {}
    if assembly_definition and assembly_definition.moving_parts:
        try:
            from worker.utils.controllers import sinusoidal

            for part in assembly_definition.moving_parts:
                if part.control:
                    if part.control.mode == MotorControlMode.SINUSOIDAL:
                        dynamic_controllers[part.part_name] = lambda t, p=part.control: (
                            sinusoidal(t, p.speed, p.frequency or 1.0)
                        )
                    elif part.control.mode == MotorControlMode.CONSTANT:
                        control_inputs[part.part_name] = part.control.speed
        except Exception as e:
            logger.warning("failed_to_load_controllers", error=str(e))

    try:
        video_path = renders_dir / "simulation.mp4" if not smoke_test_mode else None
        sim_duration = 0.5 if smoke_test_mode else 30.0
        metrics = loop.step(
            control_inputs=control_inputs,
            duration=sim_duration,
            dynamic_controllers=dynamic_controllers,
            video_path=video_path,
        )

        # WP2: T017: GPU OOM Retry Logic
        if metrics.fail_reason and "out of memory" in metrics.fail_reason.lower():
            from shared.observability.events import emit_event
            from shared.observability.schemas import GpuOomRetryEvent

            logger.warning("gpu_oom_detected_retrying_smoke_mode")

            # Emit event for observability
            emit_event(
                GpuOomRetryEvent(
                    original_particles=loop.particle_budget,
                    reduced_particles=5000,
                )
            )

            from worker.simulation.loop import SimulationLoop

            # Re-create loop with reduced budget to force backend scene rebuild
            loop = SimulationLoop(
                str(scene_path),
                component=component,
                backend_type=backend_type,
                electronics=electronics,
                objectives=objectives,
                smoke_test_mode=True,
                session_id=session_id,
                particle_budget=5000,
            )
            metrics = loop.step(
                control_inputs=control_inputs,
                duration=sim_duration,
                dynamic_controllers=dynamic_controllers,
                video_path=None,  # No video on retry
            )

        status_msg = metrics.fail_reason or (
            "Goal achieved." if metrics.success else "Simulation stable."
        )

        if not smoke_test_mode:
            render_paths = prerender_24_views(
                component,
                output_dir=str(renders_dir),
                backend_type=backend_type,
                session_id=session_id,
                scene_path=str(scene_path),
                smoke_test_mode=smoke_test_mode,
            )
            if video_path and video_path.exists():
                render_paths.append(str(video_path))
        else:
            render_paths = []

        mjcf_content = scene_path.read_text() if scene_path.exists() else None

        cost, weight = calculate_assembly_totals(
            component,
            electronics=electronics,
            cots_parts=assembly_definition.cots_parts if assembly_definition else None,
        )

        result = SimulationResult(
            success=metrics.success,
            summary=status_msg,
            render_paths=render_paths,
            mjcf_content=mjcf_content,
            stress_summaries=metrics.stress_summaries,
            stress_fields=metrics.stress_fields,
            fluid_metrics=getattr(metrics, "fluid_metrics", []),
            total_cost=cost,
            total_weight_g=weight,
            confidence=metrics.confidence,
        )

        # T023: Generate stress heatmaps and append to render_paths
        if metrics.stress_fields:
            # Save first so preview_stress can load it
            try:
                save_simulation_result(result, working_dir / "simulation_result.json")
            except Exception as e:
                logger.error(
                    "failed_to_save_simulation_result_pre_preview", error=str(e)
                )

            stress_renders = preview_stress(component, output_dir=working_dir)
            result.render_paths.extend(stress_renders)

        try:
            save_simulation_result(result, working_dir / "simulation_result.json")
        except Exception as e:
            logger.error("failed_to_save_simulation_result", error=str(e))

        return result
    except Exception as e:
        logger.error("simulation_error", error=str(e))
        return SimulationResult(success=False, summary=f"Simulation error: {e!s}")


def validate(
    component: Compound,
    build_zone: dict | None = None,
    output_dir: Path | None = None,
    session_id: str | None = None,
    smoke_test_mode: bool = False,
    particle_budget: int | None = None,
) -> tuple[bool, str | None]:
    """Verify geometric validity."""
    logger.info(
        "validate_start",
        session_id=session_id,
        smoke_test_mode=smoke_test_mode,
        particle_budget=particle_budget,
    )
    solids = component.solids()
    if len(solids) > 1:
        for i in range(len(solids)):
            for j in range(i + 1, len(solids)):
                intersection = solids[i].intersect(solids[j])
                if intersection and intersection.volume > 0.1:
                    msg = (
                        f"Geometric intersection detected "
                        f"(volume: {intersection.volume:.2f})"
                    )
                    return (False, msg)

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
        backend_type = SimulatorBackendType.GENESIS
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

        prerender_24_views(
            component,
            output_dir=renders_dir,
            backend_type=backend_type,
            session_id=session_id,
            smoke_test_mode=smoke_test_mode,
            particle_budget=particle_budget,
        )
    except Exception as e:
        logger.warning("validate_render_capture_failed", error=str(e))

    return True, None


def validate_fem_manufacturability(
    component: Compound, session_root: Path
) -> tuple[bool, str | None]:
    """Check if FEM material validation is required and if it passes."""
    obj_path = session_root / "objectives.yaml"
    if not obj_path.exists():
        return True, None

    try:
        data = yaml.safe_load(obj_path.read_text(encoding="utf-8"))
        objs = ObjectivesYaml(**data)
        if objs.physics and objs.physics.fem_enabled:
            config = load_config()
            custom_config_path = session_root / "manufacturing_config.yaml"
            if custom_config_path.exists():
                config = load_config(str(custom_config_path))

            from worker.workbenches.models import ManufacturingMethod

            val_report = validate_and_price(
                component,
                ManufacturingMethod.CNC,
                config,
                fem_required=True,
            )
            if not val_report.is_manufacturable:
                msg = "Material validation failed: " + "; ".join(
                    map(str, val_report.violations)
                )
                return False, msg
    except Exception as e:
        logger.error("fem_manufacturability_check_failed", error=str(e))
        return False, f"FEM manufacturability check failed: {e!s}"

    return True, None
