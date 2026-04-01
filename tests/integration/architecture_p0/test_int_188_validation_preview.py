import io
import json
import os
import uuid
from pathlib import Path

import httpx
import numpy as np
import pytest
import yaml
from PIL import Image

from shared.models.schemas import (
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    ForbidZone,
    MovedObject,
    ObjectivesSection,
    PhysicsConfig,
)
from shared.models.simulation import SimulationResult
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ListFilesRequest,
    PreviewDesignRequest,
    PreviewDesignResponse,
    ReadFileRequest,
    WriteFileRequest,
)
from tests.integration.agent.helpers import seed_benchmark_assembly_definition

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
AGENTS_CONFIG_PATH = Path("config/agents_config.yaml")
BENCHMARK_RENDER_DIR = "renders/benchmark_renders"
pytestmark = pytest.mark.xdist_group(name="physics_sims")


def _default_benchmark_parts():
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {"fixed": True, "material_id": "aluminum_6061"},
        }
    ]


def _build_single_box_script(material_id: str) -> str:
    return f"""
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10).move(Location((0, 0, 5)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="{material_id}", fixed=True)
    return p
"""


def _build_open_wire_script(material_id: str) -> str:
    return f"""
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Wire.make_polygon([
        (-1, -1, 0),
        (1, -1, 0),
        (1, 1, 0),
        (-1, 1, 0),
    ])
    p.label = "wire_part"
    p.metadata = PartMetadata(material_id="{material_id}", fixed=True)
    return p
"""


def _default_objectives() -> BenchmarkDefinition:
    return BenchmarkDefinition(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="sphere",
            material_id="aluminum_6061",
            start_position=(0.0, 0.0, 10.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
        benchmark_parts=_default_benchmark_parts(),
    )


def _image_has_zone_tint(
    rgb_image: np.ndarray, zone_type: str, *, minimum_pixels: int = 100
) -> bool:
    red = rgb_image[:, :, 0].astype(np.int16)
    green = rgb_image[:, :, 1].astype(np.int16)
    blue = rgb_image[:, :, 2].astype(np.int16)

    if zone_type == "goal":
        mask = (green > red + 15) & (green > blue + 15)
    elif zone_type == "forbid":
        mask = (red > green + 15) & (red > blue + 15)
    else:
        mask = (np.abs(red - green) < 18) & (np.abs(green - blue) < 18)
    return int(mask.sum()) >= minimum_pixels


async def _write_standard_validation_inputs(
    client: httpx.AsyncClient,
    headers: dict[str, str],
    *,
    material_id: str,
) -> None:
    write_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="script.py",
            content=_build_single_box_script(material_id),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert write_resp.status_code == 200, write_resp.text

    objectives_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="benchmark_definition.yaml",
            content=yaml.dump(_default_objectives().model_dump(mode="json")),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert objectives_resp.status_code == 200, objectives_resp.text


def _set_render_modalities(
    *,
    rgb: bool | None = None,
    depth: bool | None = None,
    segmentation: bool | None = None,
    rgb_axes: bool | None = None,
    rgb_edges: bool | None = None,
    depth_axes: bool | None = None,
    depth_edges: bool | None = None,
    segmentation_axes: bool | None = None,
    segmentation_edges: bool | None = None,
) -> str:
    original = AGENTS_CONFIG_PATH.read_text(encoding="utf-8")
    data = yaml.safe_load(original) or {}
    render_cfg = dict(data.get("render") or {})

    def _modality_cfg(name: str) -> dict[str, object]:
        raw_value = render_cfg.get(name)
        if isinstance(raw_value, dict):
            return dict(raw_value)
        if isinstance(raw_value, bool):
            return {"enabled": raw_value}
        return {}

    rgb_cfg = _modality_cfg("rgb")
    depth_cfg = _modality_cfg("depth")
    segmentation_cfg = _modality_cfg("segmentation")

    if rgb is not None:
        rgb_cfg["enabled"] = rgb
    if depth is not None:
        depth_cfg["enabled"] = depth
    if segmentation is not None:
        segmentation_cfg["enabled"] = segmentation
    if rgb_axes is not None:
        rgb_cfg["axes"] = rgb_axes
    if rgb_edges is not None:
        rgb_cfg["edges"] = rgb_edges
    if depth_axes is not None:
        depth_cfg["axes"] = depth_axes
    if depth_edges is not None:
        depth_cfg["edges"] = depth_edges
    if segmentation_axes is not None:
        segmentation_cfg["axes"] = segmentation_axes
    if segmentation_edges is not None:
        segmentation_cfg["edges"] = segmentation_edges

    render_cfg["rgb"] = rgb_cfg
    render_cfg["depth"] = depth_cfg
    render_cfg["segmentation"] = segmentation_cfg
    render_cfg.pop("rgb_axes", None)
    render_cfg.pop("rgb_edges", None)
    render_cfg.pop("depth_axes", None)
    render_cfg.pop("depth_edges", None)
    render_cfg.pop("segmentation_axes", None)
    render_cfg.pop("segmentation_edges", None)
    data["render"] = render_cfg
    AGENTS_CONFIG_PATH.write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )
    return original


def _image_resolution() -> tuple[int, int]:
    render_cfg = yaml.safe_load(AGENTS_CONFIG_PATH.read_text(encoding="utf-8"))[
        "render"
    ]
    image_cfg = render_cfg["image_resolution"]
    return image_cfg["width"], image_cfg["height"]


async def _write_benchmark_submit_inputs(
    client: httpx.AsyncClient,
    headers: dict[str, str],
) -> None:
    plan_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="plan.md",
            content=(
                "## Learning Objective\n"
                "- Move a sphere into the goal zone.\n"
                "## Geometry\n"
                "- Simple box environment around origin.\n"
                "## Objectives\n"
                "- Reach goal while avoiding forbid zone.\n"
            ),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert plan_resp.status_code == 200, plan_resp.text

    todo_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="todo.md",
            content="- [x] Test benchmark\n",
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert todo_resp.status_code == 200, todo_resp.text


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_uses_build123d_even_for_genesis_objectives():
    """
    INT-188: /benchmark/validate routes static preview rendering to build123d/VTK
    even when objectives request Genesis for physics simulation.
    """
    original_config = _set_render_modalities(
        rgb=True,
        depth=True,
        segmentation=True,
        rgb_axes=False,
        rgb_edges=False,
        depth_axes=False,
        depth_edges=False,
        segmentation_axes=False,
        segmentation_edges=False,
    )
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}

            script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10).move(Location((0, 0, 5)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
            write_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert write_resp.status_code == 200, write_resp.text

            objectives = BenchmarkDefinition(
                objectives=ObjectivesSection(
                    goal_zone=BoundingBox(
                        min=(10.0, -20.0, 0.0), max=(20.0, 20.0, 20.0)
                    ),
                    forbid_zones=[
                        ForbidZone(
                            name="preview_blocker",
                            min=(-20.0, -20.0, 0.0),
                            max=(-10.0, 20.0, 20.0),
                        )
                    ],
                    build_zone=BoundingBox(
                        min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)
                    ),
                ),
                physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
                simulation_bounds=BoundingBox(
                    min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
                ),
                moved_object=MovedObject(
                    label="target_box",
                    shape="sphere",
                    material_id="aluminum_6061",
                    start_position=(0.0, 0.0, 10.0),
                    runtime_jitter=(0.0, 0.0, 0.0),
                ),
                constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
                benchmark_parts=_default_benchmark_parts(),
            )
            objectives_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="benchmark_definition.yaml",
                    content=yaml.dump(objectives.model_dump(mode="json")),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert objectives_resp.status_code == 200, objectives_resp.text

            validate_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers,
                timeout=180.0,
            )
            assert validate_resp.status_code == 200, validate_resp.text
            validate_payload = validate_resp.json()
            validate_data = BenchmarkToolResponse.model_validate(validate_payload)
            assert validate_data.success, validate_data.message
            assert validate_data.artifacts is not None
            assert validate_data.artifacts.validation_results_json is not None

            benchmark_render_events = [
                event
                for event in validate_payload.get("events", [])
                if event.get("event_type") == "render_request_benchmark"
            ]
            assert benchmark_render_events, validate_payload.get("events", [])
            assert any(
                event.get("backend") == "build123d_vtk"
                and event.get("purpose") == "validation_static_preview"
                for event in benchmark_render_events
            ), benchmark_render_events

            ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert ls_resp.status_code == 200, ls_resp.text
            render_entries = ls_resp.json()
            render_names = [
                entry["name"] for entry in render_entries if not entry["is_dir"]
            ]
            png_renders = [name for name in render_names if name.endswith(".png")]
            assert png_renders, render_names
            rgb_renders = sorted(
                name
                for name in png_renders
                if not name.endswith("_depth.png")
                and not name.endswith("_segmentation.png")
            )
            depth_renders = sorted(
                name for name in png_renders if name.endswith("_depth.png")
            )
            segmentation_renders = sorted(
                name for name in png_renders if name.endswith("_segmentation.png")
            )
            assert rgb_renders, render_names
            assert len(depth_renders) == len(rgb_renders), render_names
            assert len(segmentation_renders) == len(rgb_renders), render_names

            sample_rgb_off_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{rgb_renders[0]}"
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert sample_rgb_off_resp.status_code == 200, sample_rgb_off_resp.text
            sample_rgb_off = sample_rgb_off_resp.content

            manifest_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read",
                json=ReadFileRequest(path="renders/render_manifest.json").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert manifest_resp.status_code == 200, manifest_resp.text
            manifest = json.loads(manifest_resp.json()["content"])
            depth_meta = manifest["artifacts"][
                f"{BENCHMARK_RENDER_DIR}/{depth_renders[0]}"
            ]
            assert depth_meta["depth_min_m"] is not None, depth_meta
            assert depth_meta["depth_max_m"] is not None, depth_meta
            assert (
                "Camera-space depth in meters" in depth_meta["depth_interpretation"]
            ), depth_meta
            segmentation_meta = manifest["artifacts"][
                f"{BENCHMARK_RENDER_DIR}/{segmentation_renders[0]}"
            ]
            assert segmentation_meta["modality"] == "segmentation", segmentation_meta
            assert segmentation_meta["segmentation_legend"], segmentation_meta
            first_legend_entry = segmentation_meta["segmentation_legend"][0]
            assert first_legend_entry["semantic_label"], first_legend_entry
            assert first_legend_entry["instance_id"], first_legend_entry

            zone_types = {"goal", "forbid", "build"}
            verified_zone_types: set[str] = set()
            for rgb_name, segmentation_name in zip(
                rgb_renders, segmentation_renders, strict=True
            ):
                segmentation_meta = manifest["artifacts"][
                    f"{BENCHMARK_RENDER_DIR}/{segmentation_name}"
                ]
                legend = segmentation_meta["segmentation_legend"]
                assert legend, segmentation_meta
                assert not any(
                    str(entry.get("semantic_label") or "").startswith("zone_")
                    for entry in legend
                ), legend

                rgb_resp = await client.post(
                    f"{WORKER_LIGHT_URL}/fs/read_blob",
                    json=ReadFileRequest(
                        path=f"{BENCHMARK_RENDER_DIR}/{rgb_name}"
                    ).model_dump(mode="json"),
                    headers=headers,
                )
                assert rgb_resp.status_code == 200, rgb_resp.text
                rgb_image = np.array(
                    Image.open(io.BytesIO(rgb_resp.content)).convert("RGB")
                )

                for zone_type in zone_types:
                    if zone_type in verified_zone_types:
                        continue
                    if _image_has_zone_tint(rgb_image, zone_type):
                        verified_zone_types.add(zone_type)

            assert verified_zone_types == zone_types, verified_zone_types

            _set_render_modalities(
                rgb=True,
                depth=False,
                segmentation=True,
                rgb_axes=True,
                rgb_edges=True,
                depth_axes=True,
                depth_edges=True,
                segmentation_axes=True,
                segmentation_edges=True,
            )
            session_id_on = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers_on = {"X-Session-ID": session_id_on}
            write_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert write_resp_on.status_code == 200, write_resp_on.text

            objectives_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="benchmark_definition.yaml",
                    content=yaml.dump(objectives.model_dump(mode="json")),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert objectives_resp_on.status_code == 200, objectives_resp_on.text

            validate_resp_on = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers_on,
                timeout=180.0,
            )
            assert validate_resp_on.status_code == 200, validate_resp_on.text
            validate_data_on = BenchmarkToolResponse.model_validate(
                validate_resp_on.json()
            )
            assert validate_data_on.success, validate_data_on.message

            ls_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(
                    mode="json"
                ),
                headers=headers_on,
            )
            assert ls_resp_on.status_code == 200, ls_resp_on.text
            render_entries_on = ls_resp_on.json()
            render_names_on = [
                entry["name"] for entry in render_entries_on if not entry["is_dir"]
            ]
            rgb_renders_on = sorted(
                name
                for name in render_names_on
                if name.endswith(".png")
                and not name.endswith("_depth.png")
                and not name.endswith("_segmentation.png")
            )
            assert rgb_renders_on, render_names_on

            sample_rgb_on_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{rgb_renders_on[0]}"
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert sample_rgb_on_resp.status_code == 200, sample_rgb_on_resp.text
            assert sample_rgb_off != sample_rgb_on_resp.content

            for render_name in (
                rgb_renders[0],
                depth_renders[0],
                segmentation_renders[0],
            ):
                blob_resp = await client.post(
                    f"{WORKER_LIGHT_URL}/fs/read_blob",
                    json=ReadFileRequest(
                        path=f"{BENCHMARK_RENDER_DIR}/{render_name}"
                    ).model_dump(mode="json"),
                    headers=headers,
                )
                assert blob_resp.status_code == 200, blob_resp.text
                image = Image.open(io.BytesIO(blob_resp.content)).convert("RGB")
                extrema = image.getextrema()
                assert any(high > 0 for _, high in extrema), extrema

            fail_session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            fail_headers = {"X-Session-ID": fail_session_id}
            wire_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py",
                    content=_build_open_wire_script("aluminum_6061"),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=fail_headers,
            )
            assert wire_resp.status_code == 200, wire_resp.text

            wire_objectives_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="benchmark_definition.yaml",
                    content=yaml.dump(_default_objectives().model_dump(mode="json")),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=fail_headers,
            )
            assert wire_objectives_resp.status_code == 200, wire_objectives_resp.text

            wire_validate_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=fail_headers,
                timeout=180.0,
            )
            assert wire_validate_resp.status_code == 200, wire_validate_resp.text
            wire_validate_data = BenchmarkToolResponse.model_validate(
                wire_validate_resp.json()
            )
            assert not wire_validate_data.success, wire_validate_data
            assert wire_validate_data.message is not None
            assert (
                "preview" in wire_validate_data.message.lower()
                or "render" in wire_validate_data.message.lower()
            ), wire_validate_data.message

            wire_events = wire_validate_resp.json().get("events", [])
            assert any(
                event.get("event_type") == "render_request_benchmark"
                and event.get("backend") == "build123d_vtk"
                and event.get("purpose") == "validation_static_preview"
                for event in wire_events
            ), wire_events

            wire_ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path="renders").model_dump(mode="json"),
                headers=fail_headers,
            )
            assert wire_ls_resp.status_code == 200, wire_ls_resp.text
            wire_render_names = [
                entry["name"] for entry in wire_ls_resp.json() if not entry["is_dir"]
            ]
            assert "render_manifest.json" not in wire_render_names, wire_render_names
            assert not any(name.endswith(".png") for name in wire_render_names), (
                wire_render_names
            )
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_honors_render_modality_config():
    """INT-188: validation preview respects render modality toggles in agents_config."""
    original_config = _set_render_modalities(
        rgb=True,
        depth=False,
        segmentation=True,
        rgb_axes=False,
        rgb_edges=False,
    )
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}

            script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10).move(Location((0, 0, 5)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
            write_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert write_resp.status_code == 200, write_resp.text

            objectives = BenchmarkDefinition(
                objectives=ObjectivesSection(
                    goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                    forbid_zones=[],
                    build_zone=BoundingBox(
                        min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)
                    ),
                ),
                physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
                simulation_bounds=BoundingBox(
                    min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
                ),
                moved_object=MovedObject(
                    label="target_box",
                    shape="sphere",
                    material_id="aluminum_6061",
                    start_position=(0.0, 0.0, 10.0),
                    runtime_jitter=(0.0, 0.0, 0.0),
                ),
                constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
                benchmark_parts=_default_benchmark_parts(),
            )
            objectives_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="benchmark_definition.yaml",
                    content=yaml.dump(objectives.model_dump(mode="json")),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert objectives_resp.status_code == 200, objectives_resp.text

            validate_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers,
                timeout=180.0,
            )
            assert validate_resp.status_code == 200, validate_resp.text
            validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
            assert validate_data.success, validate_data.message

            ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert ls_resp.status_code == 200, ls_resp.text
            render_entries = ls_resp.json()
            render_names = [
                entry["name"] for entry in render_entries if not entry["is_dir"]
            ]
            png_renders = [name for name in render_names if name.endswith(".png")]
            assert any(
                not name.endswith("_depth.png")
                and not name.endswith("_segmentation.png")
                for name in png_renders
            ), render_names
            assert not any(name.endswith("_depth.png") for name in png_renders), (
                render_names
            )
            assert any(name.endswith("_segmentation.png") for name in png_renders), (
                render_names
            )

            manifest_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read",
                json=ReadFileRequest(path="renders/render_manifest.json").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert manifest_resp.status_code == 200, manifest_resp.text
            manifest = json.loads(manifest_resp.json()["content"])
            assert not any(
                path.endswith("_depth.png") for path in manifest["artifacts"]
            ), manifest
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_rejects_all_disabled_modalities():
    """INT-188: validation preview fails closed when every preview modality is disabled."""
    original_config = _set_render_modalities(
        rgb=False,
        depth=False,
        segmentation=False,
        rgb_axes=True,
        rgb_edges=True,
        depth_axes=True,
        depth_edges=True,
        segmentation_axes=True,
        segmentation_edges=True,
    )
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}

            await _write_standard_validation_inputs(
                client, headers, material_id="aluminum_6061"
            )

            validate_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers,
                timeout=180.0,
            )
            assert validate_resp.status_code == 200, validate_resp.text
            validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
            assert not validate_data.success, validate_data
            assert validate_data.message is not None
            lowered_message = validate_data.message.lower()
            assert "at least one enabled render modality" in lowered_message, (
                validate_data.message
            )

            exists_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/exists",
                json=ReadFileRequest(path="renders/render_manifest.json").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert exists_resp.status_code == 200, exists_resp.text
            assert exists_resp.json()["exists"] is False, exists_resp.text
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_overlays_axes_on_depth_and_segmentation_maps():
    """INT-188: depth and segmentation previews can share the axes/edge overlays."""
    original_config = _set_render_modalities(
        rgb=False,
        depth=True,
        segmentation=True,
        rgb_axes=False,
        rgb_edges=False,
        depth_axes=False,
        depth_edges=False,
        segmentation_axes=False,
        segmentation_edges=False,
    )
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10).move(Location((0, 0, 5)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
            objectives = _default_objectives()

            session_off = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers_off = {"X-Session-ID": session_off}
            write_resp_off = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers_off,
            )
            assert write_resp_off.status_code == 200, write_resp_off.text
            objectives_resp_off = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="benchmark_definition.yaml",
                    content=yaml.dump(objectives.model_dump(mode="json")),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers_off,
            )
            assert objectives_resp_off.status_code == 200, objectives_resp_off.text

            validate_resp_off = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers_off,
                timeout=180.0,
            )
            assert validate_resp_off.status_code == 200, validate_resp_off.text
            validate_data_off = BenchmarkToolResponse.model_validate(
                validate_resp_off.json()
            )
            assert validate_data_off.success, validate_data_off.message

            ls_resp_off = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(
                    mode="json"
                ),
                headers=headers_off,
            )
            assert ls_resp_off.status_code == 200, ls_resp_off.text
            render_names_off = [
                entry["name"] for entry in ls_resp_off.json() if not entry["is_dir"]
            ]
            depth_name_off = sorted(
                name for name in render_names_off if name.endswith("_depth.png")
            )[0]
            segmentation_name_off = sorted(
                name for name in render_names_off if name.endswith("_segmentation.png")
            )[0]

            depth_off_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{depth_name_off}"
                ).model_dump(mode="json"),
                headers=headers_off,
            )
            assert depth_off_resp.status_code == 200, depth_off_resp.text
            depth_off = np.array(
                Image.open(io.BytesIO(depth_off_resp.content)).convert("RGB")
            )

            segmentation_off_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{segmentation_name_off}"
                ).model_dump(mode="json"),
                headers=headers_off,
            )
            assert segmentation_off_resp.status_code == 200, segmentation_off_resp.text
            segmentation_off = np.array(
                Image.open(io.BytesIO(segmentation_off_resp.content)).convert("RGB")
            )

            _set_render_modalities(
                rgb=False,
                depth=True,
                segmentation=True,
                depth_axes=True,
                depth_edges=True,
                segmentation_axes=True,
                segmentation_edges=True,
            )

            session_on = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers_on = {"X-Session-ID": session_on}
            write_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert write_resp_on.status_code == 200, write_resp_on.text
            objectives_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="benchmark_definition.yaml",
                    content=yaml.dump(objectives.model_dump(mode="json")),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert objectives_resp_on.status_code == 200, objectives_resp_on.text

            validate_resp_on = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers_on,
                timeout=180.0,
            )
            assert validate_resp_on.status_code == 200, validate_resp_on.text
            validate_data_on = BenchmarkToolResponse.model_validate(
                validate_resp_on.json()
            )
            assert validate_data_on.success, validate_data_on.message

            ls_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(
                    mode="json"
                ),
                headers=headers_on,
            )
            assert ls_resp_on.status_code == 200, ls_resp_on.text
            render_names_on = [
                entry["name"] for entry in ls_resp_on.json() if not entry["is_dir"]
            ]
            depth_name_on = sorted(
                name for name in render_names_on if name.endswith("_depth.png")
            )[0]
            segmentation_name_on = sorted(
                name for name in render_names_on if name.endswith("_segmentation.png")
            )[0]

            depth_on_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{depth_name_on}"
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert depth_on_resp.status_code == 200, depth_on_resp.text
            depth_on = np.array(
                Image.open(io.BytesIO(depth_on_resp.content)).convert("RGB")
            )

            segmentation_on_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{segmentation_name_on}"
                ).model_dump(mode="json"),
                headers=headers_on,
            )
            assert segmentation_on_resp.status_code == 200, segmentation_on_resp.text
            segmentation_on = np.array(
                Image.open(io.BytesIO(segmentation_on_resp.content)).convert("RGB")
            )

            assert not np.array_equal(depth_off, depth_on), (
                "depth overlay did not change"
            )
            assert not np.array_equal(segmentation_off, segmentation_on), (
                "segmentation overlay did not change"
            )
            depth_channel_delta = np.abs(
                depth_on[:, :, 0].astype(np.int16) - depth_on[:, :, 1].astype(np.int16)
            )
            assert int(np.count_nonzero(depth_channel_delta > 4)) > 0, depth_on.shape
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_reflects_material_color_in_rgb():
    """INT-188: build123d/VTK RGB previews reflect material color configuration."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        material_means: dict[str, np.ndarray] = {}

        for material_id in ("aluminum_6061", "silicone_rubber"):
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}
            await _write_standard_validation_inputs(
                client, headers, material_id=material_id
            )

            validate_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
                headers=headers,
                timeout=180.0,
            )
            assert validate_resp.status_code == 200, validate_resp.text
            validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
            assert validate_data.success, validate_data.message

            ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert ls_resp.status_code == 200, ls_resp.text
            render_entries = ls_resp.json()
            render_names = [
                entry["name"] for entry in render_entries if not entry["is_dir"]
            ]
            rgb_name = sorted(
                name
                for name in render_names
                if name.endswith(".png")
                and not name.endswith("_depth.png")
                and not name.endswith("_segmentation.png")
            )[0]
            rgb_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(
                    path=f"{BENCHMARK_RENDER_DIR}/{rgb_name}"
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert rgb_resp.status_code == 200, rgb_resp.text
            rgb_image = np.array(
                Image.open(io.BytesIO(rgb_resp.content)).convert("RGB")
            )
            height, width = rgb_image.shape[:2]
            crop = rgb_image[
                height // 3 : (2 * height) // 3, width // 3 : (2 * width) // 3
            ]
            assert crop.size > 0, rgb_name
            material_means[material_id] = crop.mean(axis=(0, 1))

        aluminum_mean = material_means["aluminum_6061"]
        silicone_mean = material_means["silicone_rubber"]

        assert max(aluminum_mean) - min(aluminum_mean) < 25, aluminum_mean.tolist()
        assert silicone_mean[0] > silicone_mean[1] + 20, silicone_mean.tolist()
        assert silicone_mean[0] > silicone_mean[2] + 20, silicone_mean.tolist()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_rejects_stale_render_manifest_bundle():
    """INT-188: stale render manifests do not satisfy benchmark submission."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
        headers = {"X-Session-ID": session_id}

        await seed_benchmark_assembly_definition(
            client,
            session_id,
            benchmark_max_unit_cost_usd=100.0,
            planner_target_max_unit_cost_usd=80.0,
        )
        await _write_benchmark_submit_inputs(client, headers)

        unique_label = "stale_manifest_box"
        write_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py",
                content=(
                    "from build123d import *\n"
                    "from shared.models.schemas import PartMetadata\n"
                    "def build():\n"
                    "    p = Box(10, 10, 10).move(Location((0, 0, 5)))\n"
                    f'    p.label = "{unique_label}"\n'
                    '    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)\n'
                    "    return p\n"
                ),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_resp.status_code == 200, write_resp.text

        objectives = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label=unique_label,
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        objectives_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="benchmark_definition.yaml",
                content=yaml.dump(objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert objectives_resp.status_code == 200, objectives_resp.text

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(mode="json"),
            headers=headers,
        )
        assert ls_resp.status_code == 200, ls_resp.text
        render_entries = ls_resp.json()
        render_names = [
            entry["name"] for entry in render_entries if not entry["is_dir"]
        ]
        png_renders = [name for name in render_names if name.endswith(".png")]
        assert png_renders, render_names
        simulation_result = SimulationResult(
            success=True,
            summary="Goal achieved in green zone.",
            render_paths=[f"{BENCHMARK_RENDER_DIR}/{name}" for name in png_renders],
        )
        simulation_result_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="simulation_result.json",
                content=simulation_result.model_dump_json(indent=2),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert simulation_result_resp.status_code == 200, simulation_result_resp.text

    manifest_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/read",
        json=ReadFileRequest(path="renders/render_manifest.json").model_dump(
            mode="json"
        ),
        headers=headers,
    )
    assert manifest_resp.status_code == 200, manifest_resp.text
    manifest_payload = json.loads(manifest_resp.json()["content"])
    assert manifest_payload["artifacts"], manifest_payload

    stale_manifest_payload = {
        "version": manifest_payload.get("version", "1.0"),
        "episode_id": manifest_payload.get("episode_id"),
        "worker_session_id": manifest_payload.get("worker_session_id"),
        "revision": "stale-manifest-revision",
        "environment_version": manifest_payload.get("environment_version"),
        "preview_evidence_paths": manifest_payload.get("preview_evidence_paths", []),
        "artifacts": manifest_payload.get("artifacts", {}),
    }
    stale_write_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="renders/render_manifest.json",
            content=json.dumps(stale_manifest_payload, indent=2),
            overwrite=True,
            bypass_agent_permissions=True,
        ).model_dump(mode="json"),
        headers={**headers, "X-System-FS-Bypass": "1"},
    )
    assert stale_write_resp.status_code == 200, stale_write_resp.text

    submit_resp = await client.post(
        f"{WORKER_HEAVY_URL}/benchmark/submit",
        json=BenchmarkToolRequest(
            script_path="script.py",
            reviewer_stage="benchmark_reviewer",
        ).model_dump(mode="json"),
        headers=headers,
        timeout=180.0,
    )
    assert submit_resp.status_code == 200, submit_resp.text
    submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
    assert not submit_data.success, submit_data
    assert submit_data.message is not None
    lowered_message = submit_data.message.lower()
    assert "revision" in lowered_message or "out of sync" in lowered_message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_simulation_preview_uses_requested_script_entrypoint_snapshot():
    """INT-188: /benchmark/simulate keeps preview evidence aligned to the requested script path."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
        headers = {"X-Session-ID": session_id}

        await _write_standard_validation_inputs(
            client, headers, material_id="aluminum_6061"
        )

        stale_script = _build_single_box_script("aluminum_6061").replace(
            "target_box", "fallback_preview_box"
        )
        stale_write_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py",
                content=stale_script,
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert stale_write_resp.status_code == 200, stale_write_resp.text

        entrypoint_write_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="alt_entrypoint.py",
                content=_build_single_box_script("aluminum_6061").replace(
                    "target_box", "requested_preview_box"
                ),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert entrypoint_write_resp.status_code == 200, entrypoint_write_resp.text

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="alt_entrypoint.py",
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success, simulate_data.message

        manifest_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path="renders/render_manifest.json").model_dump(
                mode="json"
            ),
            headers=headers,
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = json.loads(manifest_resp.json()["content"])

        segmentation_keys = sorted(
            path for path in manifest["artifacts"] if path.endswith("_segmentation.png")
        )
        assert segmentation_keys, manifest["artifacts"]
        legend_labels = {
            entry["semantic_label"]
            for entry in manifest["artifacts"][segmentation_keys[0]][
                "segmentation_legend"
            ]
        }
        assert "requested_preview_box" in legend_labels, legend_labels
        assert "fallback_preview_box" not in legend_labels, legend_labels


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "pitch,yaw",
    [(-35.0, 45.0), (-20.0, 135.0)],
    ids=["iso", "oblique"],
)
async def test_int_188_validation_preview_http_preview_route_uses_vtk_renderer(
    pitch: float, yaw: float
):
    """INT-188: /benchmark/preview renders through the build123d/VTK path."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
        headers = {"X-Session-ID": session_id}

        await _write_standard_validation_inputs(
            client, headers, material_id="aluminum_6061"
        )

        preview_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/preview",
            json=PreviewDesignRequest(
                script_path="script.py",
                orbit_pitch=pitch,
                orbit_yaw=yaw,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert preview_resp.status_code == 200, preview_resp.text
        preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
        assert preview_data.success, preview_data.message
        assert preview_data.image_path is not None
        assert preview_data.image_path.startswith(f"{BENCHMARK_RENDER_DIR}/"), (
            preview_data
        )
        assert preview_data.image_path.endswith(".png"), preview_data

        image_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read_blob",
            json=ReadFileRequest(path=preview_data.image_path).model_dump(mode="json"),
            headers=headers,
        )
        assert image_resp.status_code == 200, image_resp.text
        image = Image.open(io.BytesIO(image_resp.content)).convert("RGB")
        assert image.size == _image_resolution(), image.size
        assert np.array(image).std() > 0.0
