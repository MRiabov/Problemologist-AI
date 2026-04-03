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

from controller.agent.tools import get_engineer_planner_tools
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.agents import get_image_render_resolution
from shared.agents.config import DraftingMode
from shared.enums import AgentName
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    CostTotals,
    DraftingCallout,
    DraftingDimension,
    DraftingNote,
    DraftingSheet,
    DraftingView,
    ForbidZone,
    ManufacturedPartEstimate,
    MovedObject,
    ObjectivesSection,
    PartConfig,
    PhysicsConfig,
)
from shared.models.simulation import SimulationResult
from shared.script_contracts import (
    drafting_render_manifest_path_for_agent,
    drafting_script_paths_for_agent,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    DeleteFileRequest,
    ListFilesRequest,
    PreviewDesignRequest,
    PreviewDesignResponse,
    PreviewRenderingType,
    ReadFileRequest,
    RenderBundlePointPickRequest,
    RenderBundlePointPickResult,
    RenderBundleQueryRequest,
    RenderBundleQueryResult,
    WriteFileRequest,
)
from shared.workers.workbench_models import ManufacturingMethod
from tests.integration.agent.helpers import seed_benchmark_assembly_definition
from tests.integration.architecture_p0.test_architecture_p0 import get_bundle

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
REPO_MANUFACTURING_CONFIG = Path(
    "worker_heavy/workbenches/manufacturing_config.yaml"
).read_text(encoding="utf-8")
AGENTS_CONFIG_PATH = Path("config/agents_config.yaml")
BENCHMARK_RENDER_DIR = "renders/benchmark_renders"
BENCHMARK_MANIFEST_PATH = f"{BENCHMARK_RENDER_DIR}/render_manifest.json"
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


def _set_engineer_planner_technical_drawing_mode(mode: DraftingMode) -> str:
    original = AGENTS_CONFIG_PATH.read_text(encoding="utf-8")
    data = yaml.safe_load(original) or {}
    agents = data.setdefault("agents", {})
    engineer_planner = agents.setdefault("engineer_planner", {})
    engineer_planner["technical_drawing_mode"] = mode.value
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
    INT-188: /benchmark/validate stays geometry-only even when objectives
    request Genesis for physics simulation, while explicit preview still works.
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

        bundle64 = await get_bundle(client, session_id)

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(
                script_path="script.py",
                bundle_base64=bundle64,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_payload = validate_resp.json()
        validate_data = BenchmarkToolResponse.model_validate(validate_payload)
        assert validate_data.success, validate_data.message
        assert validate_data.artifacts is not None
        assert validate_data.artifacts.validation_results_json is not None
        assert validate_data.artifacts.render_paths == [], validate_data.artifacts
        assert not any(
            event.get("event_type") == "render_request_benchmark"
            for event in validate_payload.get("events", [])
        ), validate_payload.get("events", [])

        validate_ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path=BENCHMARK_RENDER_DIR).model_dump(mode="json"),
            headers=headers,
        )
        assert validate_ls_resp.status_code == 200, validate_ls_resp.text
        validate_render_names = [
            entry["name"] for entry in validate_ls_resp.json() if not entry["is_dir"]
        ]
        assert "render_manifest.json" not in validate_render_names, (
            validate_render_names
        )
        assert not any(name.endswith(".png") for name in validate_render_names), (
            validate_render_names
        )
        assert not any(
            name.endswith(".jpg") or name.endswith(".jpeg")
            for name in validate_render_names
        ), validate_render_names

        preview_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/preview",
            json=PreviewDesignRequest(
                script_path="script.py",
                bundle_base64=bundle64,
                orbit_pitch=-35.0,
                orbit_yaw=45.0,
                rendering_type=PreviewRenderingType.RGB,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert preview_resp.status_code == 200, preview_resp.text
        preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
        assert preview_data.success, preview_data.message
        assert preview_data.image_path is not None
        assert preview_data.render_manifest_json is not None

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
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers=fail_headers,
            timeout=180.0,
        )
        assert wire_validate_resp.status_code == 200, wire_validate_resp.text
        wire_validate_data = BenchmarkToolResponse.model_validate(
            wire_validate_resp.json()
        )
        assert wire_validate_data.success, wire_validate_data
        assert wire_validate_data.message is not None

        wire_validate_events = wire_validate_resp.json().get("events", [])
        assert not any(
            event.get("event_type") == "render_request_benchmark"
            for event in wire_validate_events
        ), wire_validate_events

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


@pytest.mark.integration_p0
@pytest.mark.xdist_group(name="physics_sims")
@pytest.mark.asyncio
async def test_int_188_engineer_planner_submit_plan_rejects_empty_drafting_manifest():
    """INT-188: planner submit_plan must fail closed when drafting preview evidence is missing."""
    original_config = _set_engineer_planner_technical_drawing_mode(DraftingMode.MINIMAL)
    session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
    episode_id = str(uuid.uuid4())
    worker_client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=WORKER_HEAVY_URL,
        controller_url=CONTROLLER_URL,
        agent_role=AgentName.ENGINEER_PLANNER,
        light_transport="http",
    )
    fs = RemoteFilesystemMiddleware(
        worker_client,
        agent_role=AgentName.ENGINEER_PLANNER,
        episode_id=episode_id,
    )
    try:
        plan_text = (
            "## Solution Overview\n"
            "A valid solution overview.\n\n"
            "## Parts List\n"
            "| Part | Qty |\n"
            "|------|-----|\n"
            "| Box  | 1   |\n\n"
            "## Assembly Strategy\n"
            "1. Step one.\n\n"
            "## Assumption Register\n"
            "- Assumption: The planner relies on source-backed inputs that must be traceable.\n\n"
            "## Detailed Calculations\n"
            "| ID | Problem / Decision | Result | Impact |\n"
            "| -- | -- | -- | -- |\n"
            "| CALC-001 | Example calculation supporting the plan | \`N/A\` | Replace this placeholder with the actual derived limit. |\n"
            "\n### CALC-001: Example calculation supporting the plan\n"
            "\n#### Problem Statement\n"
            "\nThe plan needs a traceable calculation instead of a freeform claim.\n"
            "\n#### Assumptions\n"
            "\n- \`ASSUMP-001\`: The input values are taken from the benchmark or assembly definition.\n"
            "\n#### Derivation\n"
            "\n- Compute the binding quantity from the declared inputs.\n"
            "\n#### Worst-Case Check\n"
            "\n- The derived limit must hold under the worst-case allowed inputs.\n"
            "\n#### Result\n"
            "\n- The design remains valid only if the derived limit is respected.\n"
            "\n#### Design Impact\n"
            "\n- Update the design or inputs if the calculation changes.\n"
            "\n#### Cross-References\n"
            "\n- \`plan.md#3-assembly-strategy\`\n\n"
            "## Critical Constraints / Operating Envelope\n"
            "- Constraint: The mechanism must remain inside the derived operating limits.\n\n"
            "## Cost & Weight Budget\n"
            "- Cost: $10\n"
            "- Weight: 100g\n\n"
            "## Risk Assessment\n"
            "- Risk: Low\n"
        )
        todo_text = "- [x] Step 1\n- [-] Step 2\n"

        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10.0, 10.0, 10.0), max=(20.0, 20.0, 20.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-50.0, -50.0, 0.0), max=(50.0, 50.0, 90.0)),
            ),
            benchmark_parts=[
                {
                    "part_id": "environment_fixture",
                    "label": "environment_fixture",
                    "metadata": {
                        "fixed": True,
                        "allows_engineer_interaction": True,
                        "material_id": "aluminum_6061",
                    },
                }
            ],
            simulation_bounds=BoundingBox(
                min=(-100.0, -100.0, 0.0), max=(100.0, 100.0, 100.0)
            ),
            moved_object=MovedObject(
                label="ball",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 50.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=50.0, max_weight_g=1000.0),
        )
        assembly_definition = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=50.0,
                benchmark_max_weight_g=1000.0,
                planner_target_max_unit_cost_usd=45.0,
                planner_target_max_weight_g=900.0,
            ),
            manufactured_parts=[
                ManufacturedPartEstimate(
                    part_name="environment_fixture",
                    part_id="environment_fixture",
                    manufacturing_method=ManufacturingMethod.THREE_DP,
                    material_id="aluminum_6061",
                    quantity=1,
                    part_volume_mm3=1000.0,
                    stock_bbox_mm={"x": 10.0, "y": 10.0, "z": 10.0},
                    stock_volume_mm3=1000.0,
                    removed_volume_mm3=0.0,
                    estimated_unit_cost_usd=10.0,
                )
            ],
            cots_parts=[],
            final_assembly=[
                PartConfig(name="environment_fixture", config=AssemblyPartConfig())
            ],
            totals=CostTotals(
                estimated_unit_cost_usd=30.0,
                estimated_weight_g=500.0,
                estimate_confidence="high",
            ),
        )

        drafting_script_path, drafting_technical_drawing_path = (
            drafting_script_paths_for_agent(AgentName.ENGINEER_PLANNER)
        )
        drafting_manifest_path = drafting_render_manifest_path_for_agent(
            AgentName.ENGINEER_PLANNER
        )

        await worker_client.write_file(
            "plan.md", plan_text, overwrite=True, bypass_agent_permissions=True
        )
        await worker_client.write_file(
            "todo.md", todo_text, overwrite=True, bypass_agent_permissions=True
        )
        await worker_client.write_file(
            "benchmark_definition.yaml",
            yaml.safe_dump(benchmark_definition.model_dump(mode="json")),
            overwrite=True,
            bypass_agent_permissions=True,
        )
        await worker_client.write_file(
            "assembly_definition.yaml",
            yaml.safe_dump(assembly_definition.model_dump(mode="json")),
            overwrite=True,
            bypass_agent_permissions=True,
        )
        await worker_client.write_file(
            "manufacturing_config.yaml",
            REPO_MANUFACTURING_CONFIG,
            overwrite=True,
            bypass_agent_permissions=True,
        )
        await worker_client.write_file(
            str(drafting_script_path),
            "print('preview evidence placeholder')\n",
            overwrite=True,
            bypass_agent_permissions=True,
        )
        await worker_client.write_file(
            str(drafting_technical_drawing_path),
            "from build123d import TechnicalDrawing\n\n"
            "def build():\n"
            "    drawing = TechnicalDrawing()\n"
            "    return drawing\n",
            overwrite=True,
            bypass_agent_permissions=True,
        )
        await worker_client.write_file(
            str(drafting_manifest_path),
            "{}",
            overwrite=True,
            bypass_agent_permissions=True,
        )

        tools = get_engineer_planner_tools(fs, session_id)
        submit_plan = next(tool for tool in tools if tool.__name__ == "submit_plan")

        result = await submit_plan()
        assert result["ok"] is False, result
        assert result["status"] == "rejected", result
        assert result["errors"], result
        assert any(
            "source_script_sha256" in error
            or "preview_evidence_paths" in error
            or "artifacts are empty" in error
            for error in result["errors"]
        ), result["errors"]
    finally:
        await worker_client.aclose()
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_honors_render_modality_config():
    """INT-188: preview respects render modality toggles in agents_config."""
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

            preview_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    orbit_pitch=-35.0,
                    orbit_yaw=45.0,
                    rgb=True,
                    depth=False,
                    segmentation=True,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=180.0,
            )
            assert preview_resp.status_code == 200, preview_resp.text
            preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
            assert preview_data.success, preview_data.message
            assert preview_data.manifest_path is not None

            preview_root = Path(preview_data.manifest_path).parent
            ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=str(preview_root)).model_dump(mode="json"),
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
                json=ReadFileRequest(path=preview_data.manifest_path).model_dump(
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
async def test_int_188_drafting_preview_persists_vector_sidecars():
    """INT-188: drafting preview should persist PNG, SVG, DXF, and manifest output."""
    original_config = _set_engineer_planner_technical_drawing_mode(DraftingMode.MINIMAL)
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}

            script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 6, 4).move(Location((0, 0, 2)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
            assembly_definition = AssemblyDefinition(
                version="1.0",
                constraints=AssemblyConstraints(
                    planner_target_max_unit_cost_usd=90.0,
                    planner_target_max_weight_g=900.0,
                ),
                manufactured_parts=[
                    ManufacturedPartEstimate(
                        part_name="target_box",
                        part_id="target_box",
                        manufacturing_method=ManufacturingMethod.THREE_DP,
                        material_id="aluminum_6061",
                        quantity=1,
                        part_volume_mm3=240.0,
                        stock_bbox_mm={"x": 10.0, "y": 6.0, "z": 4.0},
                        stock_volume_mm3=240.0,
                        removed_volume_mm3=0.0,
                        estimated_unit_cost_usd=1.0,
                    )
                ],
                cots_parts=[],
                final_assembly=[
                    PartConfig(name="target_box", config=AssemblyPartConfig())
                ],
                totals=CostTotals(
                    estimated_unit_cost_usd=1.0,
                    estimated_weight_g=1.0,
                    estimate_confidence="high",
                ),
                drafting=DraftingSheet(
                    sheet_id="sheet-1",
                    title="Target Box Drawing",
                    views=[
                        DraftingView(
                            view_id="front",
                            target="target_box",
                            projection="front",
                            scale=1.0,
                            datums=["A"],
                            dimensions=[
                                DraftingDimension(
                                    dimension_id="width",
                                    kind="linear",
                                    target="target_box",
                                    value=10.0,
                                    binding=True,
                                )
                            ],
                            callouts=[
                                DraftingCallout(
                                    callout_id="1",
                                    label="Target box",
                                    target="target_box",
                                )
                            ],
                            notes=[
                                DraftingNote(
                                    note_id="n1",
                                    text="Preserve the target box envelope.",
                                    critical=True,
                                )
                            ],
                        )
                    ],
                ),
            )
            write_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert write_resp.status_code == 200, write_resp.text
            assembly_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="assembly_definition.yaml",
                    content=yaml.safe_dump(
                        assembly_definition.model_dump(
                            mode="json", by_alias=True, exclude_none=True
                        ),
                        sort_keys=False,
                    ),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert assembly_resp.status_code == 200, assembly_resp.text

            preview_resp = await client.post(
                f"{WORKER_LIGHT_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    drafting=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert preview_resp.status_code == 200, preview_resp.text
            preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
            assert preview_data.success, preview_data.message
            assert preview_data.drafting is True
            assert preview_data.view_count == 1
            assert preview_data.image_path is not None
            assert preview_data.image_path.endswith(".png")
            assert preview_data.render_blobs_base64
            assert any(
                path.endswith(".svg") for path in preview_data.render_blobs_base64
            )
            assert any(
                path.endswith(".dxf") for path in preview_data.render_blobs_base64
            )
            assert preview_data.render_manifest_json is not None

            ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path="renders/engineer_renders").model_dump(
                    mode="json"
                ),
                headers=headers,
            )
            assert ls_resp.status_code == 200, ls_resp.text
            render_names = [
                entry["name"] for entry in ls_resp.json() if not entry["is_dir"]
            ]
            assert any(name.endswith(".png") for name in render_names), render_names
            assert any(name.endswith(".svg") for name in render_names), render_names
            assert any(name.endswith(".dxf") for name in render_names), render_names
            assert "render_manifest.json" in render_names, render_names
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_drafting_preview_bundle_is_inspectable():
    """INT-188: drafting preview output must be inspectable through media inspection."""
    original_config = _set_engineer_planner_technical_drawing_mode(DraftingMode.MINIMAL)
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}

            script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 6, 4).move(Location((0, 0, 2)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
            assembly_definition = AssemblyDefinition(
                version="1.0",
                constraints=AssemblyConstraints(
                    planner_target_max_unit_cost_usd=90.0,
                    planner_target_max_weight_g=900.0,
                ),
                manufactured_parts=[
                    ManufacturedPartEstimate(
                        part_name="target_box",
                        part_id="target_box",
                        manufacturing_method=ManufacturingMethod.THREE_DP,
                        material_id="aluminum_6061",
                        quantity=1,
                        part_volume_mm3=240.0,
                        stock_bbox_mm={"x": 10.0, "y": 6.0, "z": 4.0},
                        stock_volume_mm3=240.0,
                        removed_volume_mm3=0.0,
                        estimated_unit_cost_usd=1.0,
                    )
                ],
                cots_parts=[],
                final_assembly=[
                    PartConfig(name="target_box", config=AssemblyPartConfig())
                ],
                totals=CostTotals(
                    estimated_unit_cost_usd=1.0,
                    estimated_weight_g=1.0,
                    estimate_confidence="high",
                ),
                drafting=DraftingSheet(
                    sheet_id="sheet-1",
                    title="Target Box Drawing",
                    views=[
                        DraftingView(
                            view_id="front",
                            target="target_box",
                            projection="front",
                            scale=1.0,
                            datums=["A"],
                            dimensions=[
                                DraftingDimension(
                                    dimension_id="width",
                                    kind="linear",
                                    target="target_box",
                                    value=10.0,
                                    binding=True,
                                )
                            ],
                            callouts=[
                                DraftingCallout(
                                    callout_id="1",
                                    label="Target box",
                                    target="target_box",
                                )
                            ],
                            notes=[
                                DraftingNote(
                                    note_id="n1",
                                    text="Preserve the target box envelope.",
                                    critical=True,
                                )
                            ],
                        )
                    ],
                ),
            )
            write_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="script.py", content=script, overwrite=True
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert write_resp.status_code == 200, write_resp.text
            assembly_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path="assembly_definition.yaml",
                    content=yaml.safe_dump(
                        assembly_definition.model_dump(
                            mode="json", by_alias=True, exclude_none=True
                        ),
                        sort_keys=False,
                    ),
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert assembly_resp.status_code == 200, assembly_resp.text

            preview_resp = await client.post(
                f"{WORKER_LIGHT_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    drafting=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert preview_resp.status_code == 200, preview_resp.text
            preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
            assert preview_data.success, preview_data.message
            assert preview_data.image_path is not None

            fs = RemoteFilesystemMiddleware(
                WorkerClient(
                    base_url=WORKER_LIGHT_URL,
                    session_id=session_id,
                    agent_role=AgentName.ENGINEER_PLANNER,
                    light_transport="http",
                ),
                agent_role=AgentName.ENGINEER_PLANNER,
                episode_id=str(uuid.uuid4()),
            )
            inspection = await fs.inspect_media(preview_data.image_path)
            assert inspection.media_kind == "image"
            assert inspection.attached_to_model is True
            assert inspection.attached_media_count == 1
            assert inspection.render_metadata is not None
            assert inspection.render_metadata.siblings.svg.endswith(".svg")
            assert inspection.render_metadata.siblings.dxf.endswith(".dxf")
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_rejects_all_disabled_modalities():
    """INT-188: preview fails closed when every preview modality is disabled."""
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

            preview_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json={
                    "script_path": "script.py",
                    "orbit_pitch": -35.0,
                    "orbit_yaw": 45.0,
                    "rgb": False,
                    "depth": False,
                    "segmentation": False,
                },
                headers=headers,
                timeout=180.0,
            )
            assert preview_resp.status_code == 422, preview_resp.text
            assert "at least one preview modality must be enabled" in preview_resp.text

            exists_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/exists",
                json=ReadFileRequest(path=BENCHMARK_MANIFEST_PATH).model_dump(
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

            preview_resp_off = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    orbit_pitch=-35.0,
                    orbit_yaw=45.0,
                    rgb=False,
                    depth=True,
                    segmentation=True,
                ).model_dump(mode="json"),
                headers=headers_off,
                timeout=180.0,
            )
            assert preview_resp_off.status_code == 200, preview_resp_off.text
            preview_data_off = PreviewDesignResponse.model_validate(
                preview_resp_off.json()
            )
            assert preview_data_off.success, preview_data_off.message
            assert preview_data_off.manifest_path is not None

            preview_root_off = Path(preview_data_off.manifest_path).parent
            ls_resp_off = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=str(preview_root_off)).model_dump(
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

            preview_resp_on = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    orbit_pitch=-35.0,
                    orbit_yaw=45.0,
                    rgb=False,
                    depth=True,
                    segmentation=True,
                ).model_dump(mode="json"),
                headers=headers_on,
                timeout=180.0,
            )
            assert preview_resp_on.status_code == 200, preview_resp_on.text
            preview_data_on = PreviewDesignResponse.model_validate(
                preview_resp_on.json()
            )
            assert preview_data_on.success, preview_data_on.message
            assert preview_data_on.manifest_path is not None

            preview_root_on = Path(preview_data_on.manifest_path).parent
            ls_resp_on = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(path=str(preview_root_on)).model_dump(
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

            preview_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    orbit_pitch=-35.0,
                    orbit_yaw=45.0,
                    rgb=True,
                    depth=False,
                    segmentation=False,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=180.0,
            )
            assert preview_resp.status_code == 200, preview_resp.text
            preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
            assert preview_data.success, preview_data.message
            assert preview_data.manifest_path is not None

            ls_resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/ls",
                json=ListFilesRequest(
                    path=str(Path(preview_data.manifest_path).parent)
                ).model_dump(mode="json"),
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
                    path=f"{Path(preview_data.manifest_path).parent}/{rgb_name}"
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

        preview_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/preview",
            json=PreviewDesignRequest(
                script_path="script.py",
                orbit_pitch=-35.0,
                orbit_yaw=45.0,
                rgb=True,
                depth=False,
                segmentation=False,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert preview_resp.status_code == 200, preview_resp.text
        preview_data = PreviewDesignResponse.model_validate(preview_resp.json())
        assert preview_data.success, preview_data.message
        assert preview_data.manifest_path is not None

        preview_root = Path(preview_data.manifest_path).parent
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path=str(preview_root)).model_dump(mode="json"),
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
            render_paths=[f"{preview_root}/{name}" for name in png_renders],
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
            json=ReadFileRequest(path=preview_data.manifest_path).model_dump(
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
            "preview_evidence_paths": manifest_payload.get(
                "preview_evidence_paths", []
            ),
            "artifacts": manifest_payload.get("artifacts", {}),
        }
        stale_write_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path=BENCHMARK_MANIFEST_PATH,
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
            json=ReadFileRequest(path=BENCHMARK_MANIFEST_PATH).model_dump(mode="json"),
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


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_keeps_historical_bundles_in_render_index():
    """INT-188: historical preview bundles stay discoverable through the append-only index."""
    original_config = _set_render_modalities(
        rgb=True,
        depth=False,
        segmentation=False,
        rgb_axes=False,
        rgb_edges=False,
    )
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
            headers = {"X-Session-ID": session_id}

            await _write_standard_validation_inputs(
                client, headers, material_id="aluminum_6061"
            )

            first_preview_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    orbit_pitch=-35.0,
                    orbit_yaw=45.0,
                    rendering_type=PreviewRenderingType.RGB,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=180.0,
            )
            assert first_preview_resp.status_code == 200, first_preview_resp.text
            first_preview = PreviewDesignResponse.model_validate(
                first_preview_resp.json()
            )
            assert first_preview.success, first_preview.message

            second_preview_resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/preview",
                json=PreviewDesignRequest(
                    script_path="script.py",
                    orbit_pitch=-20.0,
                    orbit_yaw=135.0,
                    rendering_type=PreviewRenderingType.RGB,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=180.0,
            )
            assert second_preview_resp.status_code == 200, second_preview_resp.text
            second_preview = PreviewDesignResponse.model_validate(
                second_preview_resp.json()
            )
            assert second_preview.success, second_preview.message

            render_index_resp = await client.get(
                f"{WORKER_LIGHT_URL}/render/bundles",
                headers=headers,
            )
            assert render_index_resp.status_code == 200, render_index_resp.text
            render_index = render_index_resp.json()
            preview_entries = [
                entry
                for entry in render_index
                if entry["bundle_path"] == "renders/benchmark_renders"
            ]
            assert len(preview_entries) >= 2, preview_entries
            bundle_ids = {
                entry["bundle_id"]
                for entry in preview_entries
                if entry.get("bundle_id")
            }
            assert len(bundle_ids) >= 2, preview_entries
            assert all(
                Path(entry["manifest_path"]) == Path("renders/render_manifest.json")
                for entry in preview_entries
            )

            selected_entry = preview_entries[-1]
            bundle_id = selected_entry["bundle_id"]
            bundle_path = selected_entry["bundle_path"]

            query_reject_resp = await client.post(
                f"{WORKER_LIGHT_URL}/render/query",
                json=RenderBundleQueryRequest(
                    bundle_path=bundle_path,
                    manifest_path=second_preview.manifest_path,
                    bundle_id=f"{bundle_id}-stale",
                    limit=1,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=60.0,
            )
            assert query_reject_resp.status_code == 422, query_reject_resp.text

            query_resp = await client.post(
                f"{WORKER_LIGHT_URL}/render/query",
                json=RenderBundleQueryRequest(
                    bundle_path=bundle_path,
                    manifest_path=second_preview.manifest_path,
                    bundle_id=bundle_id,
                    limit=1,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=60.0,
            )
            assert query_resp.status_code == 200, query_resp.text
            query_result = RenderBundleQueryResult.model_validate(query_resp.json())
            assert query_result.bundle_id == bundle_id, query_result
            assert query_result.manifest_path == second_preview.manifest_path, (
                query_result
            )
            assert second_preview.artifact_path in query_result.preview_evidence_paths
            assert query_result.frames == [], query_result.frames

            image_width, image_height = get_image_render_resolution()
            pick_resp = await client.post(
                f"{WORKER_LIGHT_URL}/render/pick",
                json=RenderBundlePointPickRequest(
                    bundle_path=bundle_path,
                    manifest_path=second_preview.manifest_path,
                    bundle_id=bundle_id,
                    pixel_x=image_width // 2,
                    pixel_y=image_height // 2,
                    image_width=image_width,
                    image_height=image_height,
                ).model_dump(mode="json"),
                headers=headers,
                timeout=60.0,
            )
            assert pick_resp.status_code == 200, pick_resp.text
            pick_result = RenderBundlePointPickResult.model_validate(pick_resp.json())
            assert pick_result.bundle_id == bundle_id, pick_result
            assert pick_result.manifest_path == second_preview.manifest_path, (
                pick_result
            )
            assert pick_result.hit is True, pick_result
            assert pick_result.world_point is not None, pick_result
            assert pick_result.object_identity is not None, pick_result
            assert pick_result.object_identity.label is not None, pick_result

            bad_manifest_path = Path(second_preview.manifest_path).with_name(
                "render_manifest_scene_hash_mismatch.json"
            )
            try:
                assert second_preview.render_manifest_json is not None
                bad_manifest_data = json.loads(second_preview.render_manifest_json)
                bad_manifest_data["scene_hash"] = "deadbeef"
                write_resp = await client.post(
                    f"{WORKER_LIGHT_URL}/fs/write",
                    json=WriteFileRequest(
                        path=str(bad_manifest_path),
                        content=json.dumps(bad_manifest_data, indent=2),
                        overwrite=True,
                    ).model_dump(mode="json"),
                    headers=headers,
                    timeout=60.0,
                )
                assert write_resp.status_code == 200, write_resp.text

                mismatch_resp = await client.post(
                    f"{WORKER_LIGHT_URL}/render/pick",
                    json=RenderBundlePointPickRequest(
                        bundle_path=bundle_path,
                        manifest_path=str(bad_manifest_path),
                        bundle_id=bundle_id,
                        pixel_x=image_width // 2,
                        pixel_y=image_height // 2,
                        image_width=image_width,
                        image_height=image_height,
                    ).model_dump(mode="json"),
                    headers=headers,
                    timeout=60.0,
                )
                assert mismatch_resp.status_code == 422, mismatch_resp.text

                batch_resp = await client.post(
                    f"{WORKER_LIGHT_URL}/render/pick/batch",
                    json=[
                        RenderBundlePointPickRequest(
                            bundle_path=bundle_path,
                            manifest_path=second_preview.manifest_path,
                            bundle_id=bundle_id,
                            pixel_x=image_width // 2,
                            pixel_y=image_height // 2,
                            image_width=image_width,
                            image_height=image_height,
                        ).model_dump(mode="json"),
                        RenderBundlePointPickRequest(
                            bundle_path=bundle_path,
                            manifest_path=second_preview.manifest_path,
                            bundle_id=bundle_id,
                            pixel_x=image_width // 2,
                            pixel_y=image_height // 2,
                            image_width=image_width,
                            image_height=image_height,
                        ).model_dump(mode="json"),
                    ],
                    headers=headers,
                    timeout=60.0,
                )
                assert batch_resp.status_code == 200, batch_resp.text
                batch_results = [
                    RenderBundlePointPickResult.model_validate(item)
                    for item in batch_resp.json()
                ]
                assert len(batch_results) == 2, batch_results
                assert all(result.bundle_id == bundle_id for result in batch_results)
                assert all(result.hit for result in batch_results)
            finally:
                await client.post(
                    f"{WORKER_LIGHT_URL}/fs/delete",
                    json=DeleteFileRequest(
                        path=str(bad_manifest_path),
                        bypass_agent_permissions=True,
                    ).model_dump(mode="json"),
                    headers=headers,
                    timeout=60.0,
                )
    finally:
        AGENTS_CONFIG_PATH.write_text(original_config, encoding="utf-8")
