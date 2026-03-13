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
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ListFilesRequest,
    ReadFileRequest,
    WriteFileRequest,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
AGENTS_CONFIG_PATH = Path("config/agents_config.yaml")


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
            start_position=(0.0, 0.0, 10.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
        benchmark_parts=_default_benchmark_parts(),
    )


def _zone_entry_for_label(
    legend: list[dict[str, object]], label_prefix: str
) -> dict[str, object] | None:
    for entry in legend:
        semantic_label = str(entry.get("semantic_label") or "")
        instance_name = str(entry.get("instance_name") or "")
        if semantic_label.startswith(label_prefix) or instance_name.startswith(
            label_prefix
        ):
            return entry
    return None


def _mask_from_legend_color(
    segmentation_image: np.ndarray, legend_entry: dict[str, object]
) -> np.ndarray:
    color = np.array(legend_entry["color_rgb"], dtype=np.uint8)
    return np.all(segmentation_image == color, axis=2)


def _assert_zone_tint(
    rgb_image: np.ndarray,
    segmentation_image: np.ndarray,
    legend_entry: dict[str, object],
    zone_type: str,
) -> None:
    mask = _mask_from_legend_color(segmentation_image, legend_entry)
    assert mask.any(), legend_entry
    mean_rgb = rgb_image[mask].mean(axis=0)
    red, green, blue = mean_rgb.tolist()

    if zone_type == "goal":
        assert green > red + 15.0, mean_rgb
        assert green > blue + 15.0, mean_rgb
        return
    if zone_type == "forbid":
        assert red > green + 15.0, mean_rgb
        assert red > blue + 15.0, mean_rgb
        return

    assert abs(red - green) < 18.0, mean_rgb
    assert abs(green - blue) < 18.0, mean_rgb


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
) -> str:
    original = AGENTS_CONFIG_PATH.read_text(encoding="utf-8")
    data = yaml.safe_load(original) or {}
    render_cfg = dict(data.get("render") or {})
    if rgb is not None:
        render_cfg["rgb"] = rgb
    if depth is not None:
        render_cfg["depth"] = depth
    if segmentation is not None:
        render_cfg["segmentation"] = segmentation
    data["render"] = render_cfg
    AGENTS_CONFIG_PATH.write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )
    return original


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_uses_mujoco_even_for_genesis_objectives():
    """
    INT-188: /benchmark/validate routes static preview rendering to MuJoCo
    even when objectives request Genesis for physics simulation.
    """
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
                goal_zone=BoundingBox(min=(10.0, -20.0, 0.0), max=(20.0, 20.0, 20.0)),
                forbid_zones=[
                    ForbidZone(
                        name="preview_blocker",
                        min=(-20.0, -20.0, 0.0),
                        max=(-10.0, 20.0, 20.0),
                    )
                ],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
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
            event.get("backend") == SimulatorBackendType.MUJOCO.value
            and event.get("purpose") == "validation_static_preview"
            for event in benchmark_render_events
        ), benchmark_render_events

        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="renders").model_dump(mode="json"),
            headers=headers,
        )
        assert ls_resp.status_code == 200, ls_resp.text
        render_entries = ls_resp.json()
        render_names = [
            entry["name"] for entry in render_entries if not entry["is_dir"]
        ]
        png_renders = [name for name in render_names if name.endswith(".png")]
        assert png_renders, render_names
        assert "render_manifest.json" in render_names, render_names
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

        manifest_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path="renders/render_manifest.json").model_dump(
                mode="json"
            ),
            headers=headers,
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = json.loads(manifest_resp.json()["content"])
        segmentation_meta = manifest["artifacts"][f"renders/{segmentation_renders[0]}"]
        assert segmentation_meta["modality"] == "segmentation", segmentation_meta
        assert segmentation_meta["segmentation_legend"], segmentation_meta
        first_legend_entry = segmentation_meta["segmentation_legend"][0]
        assert first_legend_entry["semantic_label"], first_legend_entry
        assert first_legend_entry["instance_id"], first_legend_entry

        zone_checks = {
            "zone_goal": "goal",
            "zone_forbid": "forbid",
            "zone_build": "build",
        }
        verified_zones: set[str] = set()
        for rgb_name, segmentation_name in zip(
            rgb_renders, segmentation_renders, strict=True
        ):
            segmentation_meta = manifest["artifacts"][f"renders/{segmentation_name}"]
            legend = segmentation_meta["segmentation_legend"]
            if not legend:
                continue

            rgb_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(path=f"renders/{rgb_name}").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert rgb_resp.status_code == 200, rgb_resp.text
            rgb_image = np.array(
                Image.open(io.BytesIO(rgb_resp.content)).convert("RGB")
            )

            segmentation_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(path=f"renders/{segmentation_name}").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert segmentation_resp.status_code == 200, segmentation_resp.text
            segmentation_image = np.array(
                Image.open(io.BytesIO(segmentation_resp.content)).convert("RGB")
            )

            for zone_label, zone_type in zone_checks.items():
                if zone_label in verified_zones:
                    continue
                legend_entry = _zone_entry_for_label(legend, zone_label)
                if legend_entry is None:
                    continue
                _assert_zone_tint(
                    rgb_image,
                    segmentation_image,
                    legend_entry,
                    zone_type,
                )
                verified_zones.add(zone_label)

        assert verified_zones == set(zone_checks), verified_zones

        for render_name in (
            rgb_renders[0],
            depth_renders[0],
            segmentation_renders[0],
        ):
            blob_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(path=f"renders/{render_name}").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert blob_resp.status_code == 200, blob_resp.text
            image = Image.open(io.BytesIO(blob_resp.content)).convert("RGB")
            extrema = image.getextrema()
            assert any(high > 0 for _, high in extrema), extrema


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_honors_render_modality_config():
    """INT-188: validation preview respects render modality toggles in agents_config."""
    original_config = _set_render_modalities(rgb=True, depth=False, segmentation=True)
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
                json=ListFilesRequest(path="renders").model_dump(mode="json"),
                headers=headers,
            )
            assert ls_resp.status_code == 200, ls_resp.text
            render_entries = ls_resp.json()
            render_names = [
                entry["name"] for entry in render_entries if not entry["is_dir"]
            ]
            png_renders = [name for name in render_names if name.endswith(".png")]
            assert "render_manifest.json" in render_names, render_names
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
async def test_int_188_validation_preview_reflects_material_color_in_rgb():
    """INT-188: MuJoCo RGB previews reflect material color configuration."""
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
                json=ListFilesRequest(path="renders").model_dump(mode="json"),
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
            segmentation_name = sorted(
                name for name in render_names if name.endswith("_segmentation.png")
            )[0]

            rgb_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(path=f"renders/{rgb_name}").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert rgb_resp.status_code == 200, rgb_resp.text
            rgb_image = np.array(
                Image.open(io.BytesIO(rgb_resp.content)).convert("RGB")
            )

            segmentation_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/read_blob",
                json=ReadFileRequest(path=f"renders/{segmentation_name}").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert segmentation_resp.status_code == 200, segmentation_resp.text
            segmentation_image = np.array(
                Image.open(io.BytesIO(segmentation_resp.content)).convert("RGB")
            )
            mask = np.any(segmentation_image > 0, axis=2)
            assert mask.any(), segmentation_name
            material_means[material_id] = rgb_image[mask].mean(axis=0)

        aluminum_mean = material_means["aluminum_6061"]
        silicone_mean = material_means["silicone_rubber"]

        assert max(aluminum_mean) - min(aluminum_mean) < 25, aluminum_mean.tolist()
        assert silicone_mean[0] > silicone_mean[1] + 40, silicone_mean.tolist()
        assert silicone_mean[0] > silicone_mean[2] + 40, silicone_mean.tolist()
