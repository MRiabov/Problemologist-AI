import asyncio
import hashlib
import json
import os
import re
import uuid
from contextlib import suppress
from pathlib import Path

import httpx
import pytest
import yaml

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeResponse,
)
from shared.enums import (
    AgentName,
    EpisodeStatus,
    ReviewDecision,
    TerminalReason,
    TraceType,
)
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    CostTotals,
    MovedObject,
    ObjectivesSection,
    PartConfig,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    DeleteFileRequest,
    ExecuteRequest,
    ExecuteResponse,
    PlanReviewManifest,
    ReadFileRequest,
    ReadFileResponse,
    WriteFileRequest,
)
from tests.integration.agent.helpers import wait_for_benchmark_state

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
REPO_MANUFACTURING_CONFIG = Path(
    "worker_heavy/workbenches/manufacturing_config.yaml"
).read_text(encoding="utf-8")


def _default_benchmark_parts():
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {
                "fixed": True,
                "allows_engineer_interaction": True,
                "material_id": "aluminum_6061",
            },
        }
    ]


@pytest.fixture
def session_id(request):
    match = re.search(r"test_int_(\d{3})", request.node.name, re.IGNORECASE)
    prefix = f"INT-{match.group(1)}" if match else "test-gates"
    return f"{prefix}-{uuid.uuid4().hex[:8]}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


@pytest.fixture
def valid_plan():
    return """## 1. Solution Overview
A valid solution overview.

## 2. Parts List
| Part | Qty |
|------|-----|
| Box  | 1   |

## 3. Assembly Strategy
1. Step one.

## 4. Cost & Weight Budget
- Cost: $10
- Weight: 100g

## 5. Risk Assessment
- Risk: Low
"""


@pytest.fixture
def valid_todo():
    return "- [x] Step 1\n- [-] Step 2"


@pytest.fixture
def valid_objectives():
    return BenchmarkDefinition(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10.0, 10.0, 10.0), max=(20.0, 20.0, 20.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-50.0, -50.0, 0.0), max=(50.0, 50.0, 90.0)),
        ),
        benchmark_parts=_default_benchmark_parts(),
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
        constraints=Constraints(
            estimated_solution_cost_usd=33.333333333333336,
            estimated_solution_weight_g=666.6666666666666,
        ),
    )


@pytest.fixture
def valid_cost():
    return AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=50.0,
            benchmark_max_weight_g=1000.0,
            planner_target_max_unit_cost_usd=45.0,
            planner_target_max_weight_g=900.0,
        ),
        totals=CostTotals(
            estimated_unit_cost_usd=30.0,
            estimated_weight_g=500.0,
            estimate_confidence="high",
        ),
    )


@pytest.fixture
def minimal_script():
    return """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    # Box 10x10x10 centered at (0,0,5) -> Z from 0 to 10.
    # Build zone is [0, 100] in objectives.
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""


async def setup_workspace(client, headers, files):
    """Utility to setup a workspace. Overwrites if exists."""
    for path, content in files.items():
        if isinstance(content, (BenchmarkDefinition, AssemblyDefinition)):
            content_str = yaml.dump(content.model_dump(mode="json", by_alias=True))
        elif not isinstance(content, str):
            content_str = yaml.dump(content)
        else:
            content_str = content

        write_req = WriteFileRequest(
            path=path,
            content=content_str,
            overwrite=True,
        )
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_req.model_dump(mode="json"),
            headers=headers,
        )
        assert resp.status_code == 200, f"Failed to write {path}: {resp.text}"


def _parse_submit_observation_node_type(observation: str | None) -> str | None:
    if not observation:
        return None
    with suppress(Exception):
        parsed = json.loads(observation)
        if isinstance(parsed, dict):
            node_type = parsed.get("node_type")
            return str(node_type) if node_type else None
    with suppress(Exception):
        import ast

        parsed = ast.literal_eval(observation)
        if isinstance(parsed, dict):
            node_type = parsed.get("node_type")
            return str(node_type) if node_type else None
    return None


def _submit_plan_node_types_from_episode_traces(traces) -> set[str]:
    node_types: set[str] = set()
    for trace in traces or []:
        if trace.trace_type != TraceType.TOOL_START or trace.name != "submit_plan":
            continue
        metadata = trace.metadata_vars
        if metadata is None:
            continue
        parsed_node_type = _parse_submit_observation_node_type(metadata.observation)
        if parsed_node_type:
            node_types.add(parsed_node_type)
    return node_types


async def _wait_for_submit_plan_node_types(
    client: httpx.AsyncClient, episode_id: str, required_node_type: str
) -> tuple[EpisodeStatus | None, set[str]]:
    target_statuses = {EpisodeStatus.COMPLETED, EpisodeStatus.FAILED}
    status: EpisodeStatus | None = None
    submit_node_types: set[str] = set()
    for _ in range(90):
        ep_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert ep_resp.status_code == 200, ep_resp.text
        episode = EpisodeResponse.model_validate(ep_resp.json())
        status = episode.status
        submit_node_types = _submit_plan_node_types_from_episode_traces(
            episode.traces or []
        )
        if required_node_type in submit_node_types:
            break
        if status in target_statuses:
            break
        await asyncio.sleep(1)
    return status, submit_node_types


async def _wait_for_planned_after_submit_plan(
    client: httpx.AsyncClient, episode_id: str
) -> EpisodeStatus | None:
    status: EpisodeStatus | None = None
    for _ in range(90):
        ep_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert ep_resp.status_code == 200, ep_resp.text
        episode = EpisodeResponse.model_validate(ep_resp.json())
        status = episode.status
        if status in {EpisodeStatus.PLANNED, EpisodeStatus.FAILED}:
            break
        await asyncio.sleep(1)
    return status


async def _wait_for_submit_plan_node_types_benchmark(
    client: httpx.AsyncClient, session_id: str, required_node_type: str
) -> tuple[EpisodeStatus | None, set[str]]:
    episode = EpisodeResponse.model_validate(
        await wait_for_benchmark_state(
            client,
            session_id,
            timeout_s=90.0,
            terminal_statuses={
                EpisodeStatus.PLANNED,
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
            },
            predicate=lambda candidate: (
                required_node_type
                in _submit_plan_node_types_from_episode_traces(candidate.traces or [])
            ),
        )
    )
    return (
        episode.status,
        _submit_plan_node_types_from_episode_traces(episode.traces or []),
    )


async def _wait_for_planned_after_submit_plan_benchmark(
    client: httpx.AsyncClient, session_id: str
) -> EpisodeStatus | None:
    episode = EpisodeResponse.model_validate(
        await wait_for_benchmark_state(
            client,
            session_id,
            timeout_s=90.0,
            terminal_statuses={EpisodeStatus.PLANNED, EpisodeStatus.FAILED},
        )
    )
    return episode.status


async def _wait_for_benchmark_asset(
    client: httpx.AsyncClient,
    session_id: str,
    suffix: str,
    timeout_s: float = 90.0,
) -> list[str]:
    episode = EpisodeResponse.model_validate(
        await wait_for_benchmark_state(
            client,
            session_id,
            timeout_s=timeout_s,
            terminal_statuses={EpisodeStatus.FAILED},
            predicate=lambda candidate: any(
                asset.s3_path.endswith(suffix) for asset in (candidate.assets or [])
            ),
        )
    )
    return [
        asset.s3_path
        for asset in (episode.assets or [])
        if asset.s3_path.endswith(suffix)
    ]


async def _generate_ready_benchmark_session(
    client: httpx.AsyncClient, *, prompt: str
) -> str:
    req = BenchmarkGenerateRequest(prompt=prompt, backend=SimulatorBackendType.GENESIS)
    resp = await client.post(
        f"{CONTROLLER_URL}/benchmark/generate", json=req.model_dump(mode="json")
    )
    assert resp.status_code in {200, 202}, resp.text
    run_resp = BenchmarkGenerateResponse.model_validate(resp.json())
    benchmark_session_id = str(run_resp.session_id)

    planned_session = EpisodeResponse.model_validate(
        await wait_for_benchmark_state(
            client,
            benchmark_session_id,
            timeout_s=150.0,
            terminal_statuses={
                EpisodeStatus.PLANNED,
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
            },
        )
    )
    if planned_session.status == EpisodeStatus.FAILED:
        pytest.fail(
            "Benchmark generation failed during setup "
            f"(session_id={benchmark_session_id})."
        )

    if planned_session.status == EpisodeStatus.PLANNED:
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="manufacturing_config.yaml",
                content=REPO_MANUFACTURING_CONFIG,
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": benchmark_session_id},
        )
        await client.post(
            f"{CONTROLLER_URL}/benchmark/{benchmark_session_id}/confirm",
            json=ConfirmRequest(comment="Proceed").model_dump(),
        )

        approved_session = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                benchmark_session_id,
                timeout_s=300.0,
                terminal_statuses={
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                    EpisodeStatus.CANCELLED,
                },
            )
        )
        if approved_session.status != EpisodeStatus.COMPLETED:
            pytest.fail(
                "Benchmark generation failed during setup "
                f"(session_id={benchmark_session_id}, status={approved_session.status})."
            )

    return str(run_resp.episode_id)


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "plan_md_missing",
        "todo_md_missing",
        "benchmark_assembly_definition_yaml_missing",
    ]
)
@pytest.mark.asyncio
async def test_int_005_mandatory_artifacts_gate(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
    minimal_script,
):
    """INT-005: Verify rejection if mandatory artifacts are missing."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Initial: All required files except one
        base_files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": valid_objectives,
            "benchmark_assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Missing plan.md
        files = base_files.copy()
        del files["plan.md"]
        await setup_workspace(client, base_headers, files)
        delete_req = DeleteFileRequest(path="plan.md")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_req.model_dump(mode="json"),
            headers=base_headers,
        )

        submit_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "plan.md is missing" in data.message

        # 2. Missing todo.md
        files = base_files.copy()
        del files["todo.md"]
        await setup_workspace(client, base_headers, files)
        delete_todo_req = DeleteFileRequest(path="todo.md")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_todo_req.model_dump(mode="json"),
            headers=base_headers,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "todo.md is missing" in data.message

        # 3. Missing benchmark_assembly_definition.yaml
        files = base_files.copy()
        del files["benchmark_assembly_definition.yaml"]
        await setup_workspace(client, base_headers, files)
        delete_cost_req = DeleteFileRequest(path="benchmark_assembly_definition.yaml")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_cost_req.model_dump(mode="json"),
            headers=base_headers,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "benchmark_assembly_definition.yaml is missing" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_005_engineer_planner_flow_emits_submit_plan_trace():
    """INT-005: Engineer planner must emit explicit submit_plan TOOL_START before completion."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        benchmark_session_id = await _generate_ready_benchmark_session(
            client, prompt="INT-005 benchmark setup for engineer planner trace test."
        )
        session_id = f"INT-005-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-005 engineer planner submission trace contract.",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_PLANNER,
            metadata_vars={"benchmark_id": benchmark_session_id},
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run", json=req.model_dump(mode="json")
        )
        assert resp.status_code == 202, resp.text
        run_resp = AgentRunResponse.model_validate(resp.json())
        episode_id = str(run_resp.episode_id)

        status, submit_node_types = await _wait_for_submit_plan_node_types(
            client, episode_id, AgentName.ENGINEER_PLANNER.value
        )
        assert AgentName.ENGINEER_PLANNER.value in submit_node_types, (
            "Expected submit_plan TOOL_START trace with node_type=engineer_planner "
            f"in engineer planner flow. Observed node_types={sorted(submit_node_types)}, status={status}"
        )
        post_submit_status = await _wait_for_planned_after_submit_plan(
            client, episode_id
        )
        assert post_submit_status != EpisodeStatus.FAILED, (
            "Engineer planner reached FAILED after submit_plan; expected PLANNED."
        )
        assert post_submit_status == EpisodeStatus.PLANNED, (
            f"Expected engineer planner to reach PLANNED after submit_plan, got {post_submit_status}."
        )

        manifest_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode_id}/assets/.manifests/engineering_plan_review_manifest.json"
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = PlanReviewManifest.model_validate_json(manifest_resp.text)
        assert "manufacturing_config.yaml" in manifest.artifact_hashes, manifest

        config_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode_id}/assets/manufacturing_config.yaml"
        )
        assert config_resp.status_code == 200, config_resp.text
        expected_hash = hashlib.sha256(config_resp.text.encode("utf-8")).hexdigest()
        assert manifest.artifact_hashes["manufacturing_config.yaml"] == expected_hash


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_113_electronics_planner_flow_emits_submit_plan_trace():
    """INT-113: Electronics planner must emit explicit submit_plan TOOL_START before completion."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        benchmark_session_id = await _generate_ready_benchmark_session(
            client,
            prompt="INT-113 benchmark setup for electronics planner trace test.",
        )
        session_id = f"INT-113-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-113 electronics planner submission trace contract.",
            session_id=session_id,
            agent_name=AgentName.ELECTRONICS_PLANNER,
            metadata_vars={"benchmark_id": benchmark_session_id},
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run", json=req.model_dump(mode="json")
        )
        assert resp.status_code == 202, resp.text
        run_resp = AgentRunResponse.model_validate(resp.json())
        episode_id = str(run_resp.episode_id)

        status, submit_node_types = await _wait_for_submit_plan_node_types(
            client, episode_id, AgentName.ELECTRONICS_PLANNER.value
        )
        assert AgentName.ELECTRONICS_PLANNER.value in submit_node_types, (
            "Expected submit_plan TOOL_START trace with node_type=electronics_planner "
            f"in electronics planner flow. Observed node_types={sorted(submit_node_types)}, status={status}"
        )
        post_submit_status = await _wait_for_planned_after_submit_plan(
            client, episode_id
        )
        assert post_submit_status != EpisodeStatus.FAILED, (
            "Electronics planner reached FAILED after submit_plan; expected PLANNED."
        )
        assert post_submit_status == EpisodeStatus.PLANNED, (
            f"Expected electronics planner to reach PLANNED after submit_plan, got {post_submit_status}."
        )


@pytest.mark.integration_p0
# Benchmark planner traces can include fail-closed handover rejection signatures
# from guard contracts while the planner stage still reaches PLANNED.
@pytest.mark.allow_backend_errors(
    regexes=[
        "integrated_validation_error",
        "prior_validation_missing",
    ]
)
@pytest.mark.asyncio
async def test_int_114_benchmark_planner_flow_emits_submit_plan_trace():
    """INT-114: Benchmark planner must submit to plan review before reaching PLANNED."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        req = BenchmarkGenerateRequest(
            prompt="INT-114 benchmark planner submission trace contract.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/benchmark/generate", json=req.model_dump(mode="json")
        )
        assert resp.status_code in {200, 202}, resp.text
        run_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        episode_id = str(run_resp.episode_id)

        status, submit_node_types = await _wait_for_submit_plan_node_types_benchmark(
            client, episode_id, AgentName.BENCHMARK_PLANNER.value
        )
        assert AgentName.BENCHMARK_PLANNER.value in submit_node_types, (
            "Expected submit_plan TOOL_START trace with node_type=benchmark_planner "
            f"in benchmark planner flow. Observed node_types={sorted(submit_node_types)}, status={status}"
        )
        plan_review_manifest_paths = await _wait_for_benchmark_asset(
            client, episode_id, "benchmark_plan_review_manifest.json"
        )
        assert plan_review_manifest_paths, (
            "Expected benchmark plan review manifest after planner submit_plan. "
            f"episode_id={episode_id}"
        )
        plan_review_paths = await _wait_for_benchmark_asset(
            client, episode_id, "benchmark-plan-review-decision-round-1.yaml"
        )
        assert plan_review_paths, (
            "Expected persisted benchmark plan review decision file before PLANNED. "
            f"episode_id={episode_id}"
        )
        plan_review_comment_paths = await _wait_for_benchmark_asset(
            client, episode_id, "benchmark-plan-review-comments-round-1.yaml"
        )
        assert plan_review_comment_paths, (
            "Expected persisted benchmark plan review comments file before PLANNED. "
            f"episode_id={episode_id}"
        )
        episode_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        assert episode_resp.status_code == 200, episode_resp.text
        episode_data = EpisodeResponse.model_validate(episode_resp.json())
        artifact_paths = [a.s3_path for a in (episode_data.assets or [])]

        plan_paths = [p for p in artifact_paths if p.endswith("plan.md")]
        assert plan_paths, f"plan.md missing. Artifacts: {artifact_paths}"
        plan_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode_id}/assets/{plan_paths[0]}"
        )
        assert plan_resp.status_code == 200, plan_resp.text
        plan_text = plan_resp.text.lower()
        assert "gravity" in plan_text
        assert "rigid-body" in plan_text

        benchmark_definition_paths = [
            p for p in artifact_paths if p.endswith("benchmark_definition.yaml")
        ]
        assert benchmark_definition_paths, (
            f"benchmark_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        benchmark_definition_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode_id}/assets/{benchmark_definition_paths[0]}"
        )
        assert benchmark_definition_resp.status_code == 200, (
            benchmark_definition_resp.text
        )
        benchmark_definition = BenchmarkDefinition.model_validate(
            yaml.safe_load(benchmark_definition_resp.text)
        )
        assert benchmark_definition.physics.fem_enabled is False
        assert benchmark_definition.fluids == []
        assert benchmark_definition.objectives.fluid_objectives == []
        assert benchmark_definition.objectives.stress_objectives == []
        assert benchmark_definition.electronics_requirements is None
        assert benchmark_definition.moved_object.material_id

        assembly_paths = [
            p
            for p in artifact_paths
            if p.endswith("benchmark_assembly_definition.yaml")
        ]
        assert assembly_paths, (
            f"benchmark_assembly_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        assembly_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode_id}/assets/{assembly_paths[0]}"
        )
        assert assembly_resp.status_code == 200, assembly_resp.text
        benchmark_assembly_definition = AssemblyDefinition.model_validate(
            yaml.safe_load(assembly_resp.text)
        )
        assert benchmark_assembly_definition.manufactured_parts == []
        assert benchmark_assembly_definition.cots_parts == []
        assert benchmark_assembly_definition.final_assembly == []

        post_submit_status = await _wait_for_planned_after_submit_plan_benchmark(
            client, episode_id
        )
        assert post_submit_status != EpisodeStatus.FAILED, (
            "Benchmark plan-review flow reached FAILED after submit_plan; expected PLANNED."
        )
        assert post_submit_status == EpisodeStatus.PLANNED, (
            "Expected benchmark plan-review approval to reach PLANNED, "
            f"got {post_submit_status}."
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_204_benchmark_plan_reviewer_inspects_latest_revision_renders_before_approval():
    """INT-204: Benchmark plan reviewer must inspect latest-revision renders before approval."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        req = BenchmarkGenerateRequest(
            prompt="INT-204 benchmark plan reviewer render evidence approval.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/benchmark/generate", json=req.model_dump(mode="json")
        )
        assert resp.status_code in {200, 202}, resp.text
        run_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = str(run_resp.session_id)
        episode_id = str(run_resp.episode_id)

        latest_episode = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                session_id,
                timeout_s=120.0,
                terminal_statuses=set(),
                predicate=lambda candidate: (
                    candidate.status == EpisodeStatus.PLANNED
                    and any(
                        asset.s3_path.endswith(
                            "benchmark-plan-review-decision-round-1.yaml"
                        )
                        for asset in (candidate.assets or [])
                    )
                    and any(
                        asset.s3_path.endswith(
                            "benchmark-plan-review-comments-round-1.yaml"
                        )
                        for asset in (candidate.assets or [])
                    )
                ),
            )
        )
        status = latest_episode.status
        artifact_paths = [a.s3_path for a in (latest_episode.assets or [])]
        plan_review_decision_paths = [
            path
            for path in artifact_paths
            if path.endswith("benchmark-plan-review-decision-round-1.yaml")
        ]
        plan_review_comment_paths = [
            path
            for path in artifact_paths
            if path.endswith("benchmark-plan-review-comments-round-1.yaml")
        ]

        assert status == EpisodeStatus.PLANNED, (
            f"Expected benchmark plan reviewer approval to reach PLANNED, got {status}."
        )

        episode_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        assert episode_resp.status_code == 200, episode_resp.text
        episode_data = EpisodeResponse.model_validate(episode_resp.json())
        traces = episode_data.traces or []

        inspect_media_traces = [
            trace
            for trace in traces
            if trace.trace_type.value == "TOOL_START" and trace.name == "inspect_media"
        ]
        assert len(inspect_media_traces) >= 2, (
            "Benchmark plan reviewer must inspect the latest revision render bundle."
        )

        review_traces = [trace for trace in traces if trace.name == "review_decision"]
        assert review_traces, "review_decision event missing for benchmark plan review."
        latest_review_trace = max(review_traces, key=lambda trace: trace.id)
        assert latest_review_trace.metadata_vars is not None
        assert latest_review_trace.metadata_vars.decision == ReviewDecision.APPROVED
        assert any(
            trace.id < latest_review_trace.id for trace in inspect_media_traces
        ), "inspect_media must occur before the final benchmark plan review decision."

        assert plan_review_decision_paths, (
            f"benchmark plan review decision file missing. Artifacts: {artifact_paths}"
        )
        assert plan_review_comment_paths, (
            f"benchmark plan review comments file missing. Artifacts: {artifact_paths}"
        )

        comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode_id}/assets/{plan_review_comment_paths[0]}"
        )
        assert comments_resp.status_code == 200, comments_resp.text
        comments = yaml.safe_load(comments_resp.text)
        assert comments["summary"].startswith("APPROVED:"), comments
        assert comments["checklist"]["render_count"] == 2
        assert comments["checklist"]["visual_inspection_satisfied"] is True
        assert comments["checklist"]["latest_revision_verified"] is True
        assert "review_manifest_revision" in comments["checklist"]


@pytest.mark.integration_p0
# INT-006 intentionally exercises invalid plan structure paths; both the
# high-level gate and section-level reason signatures are expected.
@pytest.mark.allow_backend_errors(
    regexes=[
        "plan_md_invalid",
        "plan_md_missing_sections",
    ]
)
@pytest.mark.asyncio
async def test_int_006_plan_structure_validation(
    session_id, base_headers, valid_todo, valid_objectives, valid_cost, minimal_script
):
    """INT-006: Verify plan.md structural requirements."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "todo.md": valid_todo,
            "benchmark_definition.yaml": valid_objectives,
            "benchmark_assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Missing required heading
        invalid_plan = "## 1. Solution Overview\nNo other headings."
        await setup_workspace(
            client, base_headers, {**base_files, "plan.md": invalid_plan}
        )
        submit_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "plan.md invalid" in data.message
        assert "Missing required section" in data.message

        # 2. Parts List missing table/bullets
        invalid_plan = """## 1. Solution Overview
Overview.
## 2. Parts List
Just some text here, no list or table.
## 3. Assembly Strategy
1. Step
## 4. Cost & Weight Budget
- Cost: 0
## 5. Risk Assessment
- Risk: None
"""
        await setup_workspace(
            client, base_headers, {**base_files, "plan.md": invalid_plan}
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "Parts List must contain a bullet list or table" in data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors("todo_md_invalid")
@pytest.mark.asyncio
async def test_int_007_todo_integrity(
    session_id, base_headers, valid_plan, valid_objectives, valid_cost, minimal_script
):
    """INT-007: Verify todo.md integrity (all items completed or skipped)."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "plan.md": valid_plan,
            "benchmark_definition.yaml": valid_objectives,
            "benchmark_assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Invalid checkbox format
        invalid_todo = "- [?] What is this?"
        await setup_workspace(
            client, base_headers, {**base_files, "todo.md": invalid_todo}
        )
        submit_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "todo.md invalid" in data.message
        assert "invalid checkbox" in data.message

        # 2. Uncompleted item at submission
        uncompleted_todo = "- [ ] I forgot to finish this"
        await setup_workspace(
            client, base_headers, {**base_files, "todo.md": uncompleted_todo}
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "must be completed or skipped" in data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_definition_yaml_invalid",
        "benchmark_definition_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_008_objectives_validation(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_cost,
    minimal_script,
    valid_objectives,
):
    """INT-008: Verify benchmark_definition.yaml schema and template detection."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Template placeholders present (e.g., x_min)
        template_content = valid_objectives.model_dump(mode="json")
        template_content["objectives"]["goal_zone"]["min"] = [
            "x_min",
            "y_min",
            "z_min",
        ]
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "benchmark_definition.yaml": template_content},
        )
        submit_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "template placeholders" in data.message

        # 2. Schema violation (wrong type)
        invalid_obj = valid_objectives.model_dump(mode="json")
        invalid_obj["objectives"]["goal_zone"]["min"] = "not_a_list"
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "benchmark_definition.yaml": invalid_obj},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "benchmark_definition.yaml invalid" in data.message

        # 3. Unknown extra fields must fail closed (top-level and nested)
        extra_obj = valid_objectives.model_dump(mode="json")
        extra_obj["unknown_top_level"] = "forbidden"
        extra_obj["objectives"]["goal_zone"]["unexpected_key"] = 123
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "benchmark_definition.yaml": extra_obj},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "benchmark_definition.yaml invalid" in data.message
        assert "extra inputs are not permitted" in data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_assembly_definition_yaml_invalid",
        "cost_estimation_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_009_cost_estimation_validation(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, minimal_script
):
    """INT-009: Verify benchmark_assembly_definition.yaml schema and placeholders."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": valid_objectives,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "solution.py": minimal_script,
        }

        # 1. Template placeholders ([implement here])
        template_cost = "version: '1.0'\ntotals:\n  estimated_unit_cost_usd: 10.0\n  note: [implement here]"
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "benchmark_assembly_definition.yaml": template_cost},
        )
        submit_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "template placeholders" in data.message

        # 2. Schema violation (missing required field)
        invalid_cost = {
            "version": "1.0",
            "totals": {},
        }  # Missing estimated_unit_cost_usd
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "benchmark_assembly_definition.yaml": invalid_cost},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "benchmark_assembly_definition.yaml invalid" in data.message

        # 3. Unknown extra fields must fail closed (top-level and nested)
        extra_cost = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 50.0,
                "benchmark_max_weight_g": 1000.0,
                "planner_target_max_unit_cost_usd": 45.0,
                "planner_target_max_weight_g": 900.0,
                "unknown_constraint_key": True,
            },
            "totals": {
                "estimated_unit_cost_usd": 30.0,
                "estimated_weight_g": 500.0,
                "estimate_confidence": "high",
            },
            "unknown_top_level": "forbidden",
        }
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "benchmark_assembly_definition.yaml": extra_cost},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "benchmark_assembly_definition.yaml invalid" in data.message
        assert "extra inputs are not permitted" in data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_assembly_definition_yaml_invalid",
        "cost_estimation_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_011_planner_caps_enforcement(session_id, base_headers):
    """INT-011: Verify handoff blockage when planner caps exceed benchmark limits."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        req = AgentRunRequest(
            task="INT-011 benchmark planner caps under benchmark caps.",
            session_id=session_id,
            agent_name=AgentName.BENCHMARK_PLANNER,
            start_node=AgentName.BENCHMARK_PLANNER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run", json=req.model_dump(mode="json")
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                str(run.episode_id),
                timeout_s=150.0,
                terminal_statuses={EpisodeStatus.PLANNED, EpisodeStatus.FAILED},
            )
        )
        metadata = episode.metadata_vars
        assert metadata is not None
        assert episode.status == EpisodeStatus.PLANNED
        assert metadata.terminal_reason == TerminalReason.HANDOFF_INVARIANT_VIOLATION
        assert metadata.failure_class == "AGENT_SEMANTIC_FAILURE"
        expected_validation_log = (
            "planner_structural: benchmark_assembly_definition.yaml: "
            "assembly_definition.constraints.planner_target_max_unit_cost_usd "
            "(60.00) must be less than or equal to "
            "benchmark_definition.constraints.estimated_solution_cost_usd * 1.5 "
            "(50.00)"
        )
        assert expected_validation_log in (metadata.validation_logs or []), (
            metadata.validation_logs
        )

        traces = episode.traces or []
        submit_plan_traces = [
            trace
            for trace in traces
            if trace.trace_type == TraceType.TOOL_START and trace.name == "submit_plan"
        ]
        assert len(submit_plan_traces) >= 1, (
            "Benchmark workflow must attempt submit_plan before planner-cap rejection."
        )
        assert not any(trace.name == "benchmark_plan_reviewer" for trace in traces), (
            "Benchmark plan reviewer should not start after planner-cap rejection."
        )
        assert not any(trace.name == "benchmark_coder" for trace in traces), (
            "Benchmark coder should not start after planner-cap rejection."
        )


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=["immutability_violation", "benchmark_definition_yaml_modified"]
)
@pytest.mark.asyncio
async def test_int_015_engineer_handover_immutability(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
    minimal_script,
):
    """INT-015: Verify immutability of benchmark_definition.yaml during handover."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await client.post(f"{WORKER_LIGHT_URL}/git/init", headers=base_headers)

        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": valid_objectives,
            "benchmark_assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }
        await setup_workspace(client, base_headers, files)

        # Baseline commit
        from shared.workers.schema import GitCommitRequest

        commit_req = GitCommitRequest(message="Initial benchmark")
        await client.post(
            f"{WORKER_LIGHT_URL}/git/commit",
            json=commit_req.model_dump(mode="json"),
            headers=base_headers,
        )

        # Cheat: modify benchmark_definition.yaml
        modified_objectives = valid_objectives.model_copy(deep=True)
        modified_objectives.constraints.estimated_solution_cost_usd = 1000.0
        await setup_workspace(
            client, base_headers, {"benchmark_definition.yaml": modified_objectives}
        )
        # Record validation
        val_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "benchmark_definition.yaml violation" in data.message
        assert "has been modified" in data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(regexes=["environment_attachment_contract_invalid"])
@pytest.mark.asyncio
async def test_int_018_submit_handoff_rejects_forbidden_environment_drilling(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
):
    """INT-018: submit handoff must fail closed on forbidden benchmark drilling."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        relaxed_objectives = valid_objectives.model_copy(deep=True)
        relaxed_objectives.constraints.estimated_solution_cost_usd = 500.0
        relaxed_objectives.constraints.estimated_solution_weight_g = 10000.0
        relaxed_objectives.benchmark_parts = [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "allows_engineer_interaction": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["fastener"],
                        "notes": "Fastener mounting without drilling only.",
                    },
                },
            }
        ]

        invalid_drilling_cost = valid_cost.model_copy(deep=True)
        invalid_drilling_cost.environment_drill_operations = [
            {
                "target_part_id": "environment_fixture",
                "hole_id": "mount_left",
                "diameter_mm": 4.0,
                "depth_mm": 8.0,
                "quantity": 1,
                "notes": "Planner requests a drilled fastener hole",
            }
        ]

        goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": relaxed_objectives,
            "benchmark_assembly_definition.yaml": invalid_drilling_cost,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "script.py": goal_script,
        }
        await setup_workspace(client, base_headers, files)

        submit_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert not submit_data.success
        assert "attachment contract" in submit_data.message.lower()
        assert "drilling is not allowed" in submit_data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "cost_estimation_yaml_invalid",
        "benchmark_assembly_definition_yaml_invalid",
    ]
)
@pytest.mark.asyncio
async def test_int_010_submit_handoff_rejects_missing_benchmark_drilling_cost(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
):
    """INT-010: submit handoff must reject totals that omit static benchmark drilling cost."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        relaxed_objectives = valid_objectives.model_copy(deep=True)
        relaxed_objectives.constraints.estimated_solution_cost_usd = 500.0
        relaxed_objectives.constraints.estimated_solution_weight_g = 10000.0
        relaxed_objectives.benchmark_parts = [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "allows_engineer_interaction": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["fastener"],
                        "drill_policy": {
                            "allowed": True,
                            "max_hole_count": 4,
                            "diameter_range_mm": [3.0, 6.0],
                            "max_depth_mm": 12.0,
                        },
                    },
                },
            }
        ]

        invalid_cost = valid_cost.model_copy(deep=True)
        invalid_cost.constraints.planner_target_max_unit_cost_usd = 10.0
        invalid_cost.totals.estimated_unit_cost_usd = 2.0
        invalid_cost.environment_drill_operations = [
            {
                "target_part_id": "environment_fixture",
                "hole_id": "mount_left",
                "diameter_mm": 4.0,
                "depth_mm": 8.0,
                "quantity": 2,
                "notes": "Two drilled holes should incur static benchmark drilling cost.",
            }
        ]

        goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": relaxed_objectives,
            "benchmark_assembly_definition.yaml": invalid_cost,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "script.py": goal_script,
        }
        await setup_workspace(client, base_headers, files)

        submit_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert not submit_data.success
        assert (
            "benchmark_assembly_definition.yaml invalid" in submit_data.message.lower()
        )
        assert "benchmark drilling cost" in submit_data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(regexes=["environment_attachment_contract_invalid"])
@pytest.mark.asyncio
async def test_int_023_submit_handoff_rejects_forbidden_benchmark_attachment_joint(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
):
    """INT-023: submit handoff must reject fastener joints to non-attachable benchmark parts."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        relaxed_objectives = valid_objectives.model_copy(deep=True)
        relaxed_objectives.constraints.estimated_solution_cost_usd = 500.0
        relaxed_objectives.constraints.estimated_solution_weight_g = 10000.0
        relaxed_objectives.benchmark_parts = [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["none"],
                        "notes": "This fixture is explicitly non-attachable.",
                    },
                },
            }
        ]

        invalid_cost = valid_cost.model_copy(deep=True)
        invalid_cost.final_assembly = [
            {
                "subassembly_id": "mounting",
                "parts": [
                    {
                        "name": "engineer_bracket",
                        "config": {"dofs": []},
                    }
                ],
                "joints": [
                    {
                        "joint_id": "fixture_mount",
                        "parts": ["engineer_bracket", "environment_fixture"],
                        "type": "fastener_joint",
                    }
                ],
            }
        ]

        goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": relaxed_objectives,
            "benchmark_assembly_definition.yaml": invalid_cost,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "script.py": goal_script,
        }
        await setup_workspace(client, base_headers, files)

        submit_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert not submit_data.success
        assert "attachment contract" in submit_data.message.lower()
        assert (
            "must declare allows_engineer_interaction: true"
            in submit_data.message.lower()
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_019_hard_constraints_gates(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, valid_cost
):
    """INT-019: benchmark submit should not enforce engineering cost/weight caps."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Keep geometry valid/simulatable and use very tight caps that would fail
        # engineering submit; benchmark submit must still pass.
        expensive_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(10, 10, 10).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""
        tight_objectives = valid_objectives.model_copy(deep=True)
        tight_objectives.constraints.estimated_solution_cost_usd = 0.01
        tight_objectives.constraints.estimated_solution_weight_g = 0.1
        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": tight_objectives,
            "benchmark_assembly_definition.yaml": valid_cost,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "script.py": expensive_script,
        }
        await setup_workspace(client, base_headers, files)

        val_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success, data.message
        assert data.artifacts is not None
        benchmark_manifest = data.artifacts.review_manifests_json.get(
            ".manifests/benchmark_review_manifest.json"
        )
        assert benchmark_manifest is not None
        parsed_manifest = json.loads(benchmark_manifest)
        assert (
            parsed_manifest.get("reviewer_stage") == AgentName.BENCHMARK_REVIEWER.value
        )


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_assembly_definition_yaml_invalid",
        "cost_estimation_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_010_planner_pricing_script_integration(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, minimal_script
):
    """INT-010: Verify validate_costing_and_price block when over caps."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Keep invalid payload as raw data so schema validation happens in the endpoint path.
        invalid_cost = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 50.0,
                "benchmark_max_weight_g": 1000.0,
                "planner_target_max_unit_cost_usd": 45.0,
                "planner_target_max_weight_g": 900.0,
            },
            "totals": {
                "estimated_unit_cost_usd": 55.0,  # OVER PLANNER CAP
                "estimated_weight_g": 500.0,
                "estimate_confidence": "high",
            },
        }
        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": valid_objectives,
            "benchmark_assembly_definition.yaml": invalid_cost,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "solution.py": minimal_script,
        }
        await setup_workspace(client, base_headers, files)
        # Record validation
        val_req = BenchmarkToolRequest(
            script_path="solution.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "benchmark_assembly_definition.yaml invalid" in data.message
        assert "exceeds target" in data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_010_validate_and_price_adds_benchmark_drilling_cost(
    session_id, base_headers
):
    """INT-010: validate_and_price must write static benchmark drilling cost into totals."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        assembly_definition = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 50.0,
                "benchmark_max_weight_g": 1000.0,
                "planner_target_max_unit_cost_usd": 45.0,
                "planner_target_max_weight_g": 900.0,
            },
            "environment_drill_operations": [
                {
                    "target_part_id": "fixture_a",
                    "hole_id": "mount_left",
                    "diameter_mm": 4.0,
                    "depth_mm": 8.0,
                    "quantity": 2,
                }
            ],
            "totals": {
                "estimated_unit_cost_usd": 0.0,
                "estimated_weight_g": 100.0,
                "estimate_confidence": "medium",
            },
        }
        custom_config = {
            "benchmark_operations": {
                "drilling": {
                    "cost_per_hole_usd": 2.0,
                }
            }
        }

        await setup_workspace(
            client,
            base_headers,
            {
                "benchmark_assembly_definition.yaml": assembly_definition,
                "manufacturing_config.yaml": custom_config,
            },
        )

        exec_resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=(
                    "python "
                    "/home/maksym/Work/proj/Problemologist/Problemologist-AI/"
                    "skills/manufacturing-knowledge/scripts/validate_and_price.py"
                ),
                timeout=60,
            ).model_dump(mode="json"),
            headers=base_headers,
        )
        assert exec_resp.status_code == 200, exec_resp.text
        exec_data = ExecuteResponse.model_validate(exec_resp.json())
        assert exec_data.exit_code == 0, exec_data.stderr
        assert "Total Estimated Cost: $4.00" in exec_data.stdout

        read_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path="benchmark_assembly_definition.yaml").model_dump(
                mode="json"
            ),
            headers=base_headers,
        )
        assert read_resp.status_code == 200, read_resp.text
        updated_assembly = AssemblyDefinition.model_validate(
            yaml.safe_load(ReadFileResponse.model_validate(read_resp.json()).content)
        )
        assert updated_assembly.totals.estimated_unit_cost_usd == pytest.approx(4.0)


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "submission_cost_limit_exceeded",
        "requested quantity 1",
        "Unit cost at requested quantity",
    ]
)
@pytest.mark.asyncio
async def test_int_010_handoff_rejects_low_quantity_that_only_passes_at_volume(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives
):
    """INT-010: the handoff gate must evaluate manufacturability at the requested quantity."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        objectives = valid_objectives.model_copy(deep=True)
        objectives.constraints.target_quantity = 1
        objectives.constraints.estimated_solution_cost_usd = 40.0
        objectives.constraints.estimated_solution_weight_g = 1000.0

        assembly_definition = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 40.0,
                "benchmark_max_weight_g": 1000.0,
                "planner_target_max_unit_cost_usd": 35.0,
                "planner_target_max_weight_g": 900.0,
            },
            "manufactured_parts": [
                {
                    "part_name": "qty_probe",
                    "part_id": "qty_probe",
                    "manufacturing_method": "CNC",
                    "material_id": "aluminum_6061",
                    "quantity": 1,
                    "part_volume_mm3": 1000.0,
                    "stock_bbox_mm": {"x": 10.0, "y": 10.0, "z": 10.0},
                    "stock_volume_mm3": 1000.0,
                    "removed_volume_mm3": 0.0,
                    "estimated_unit_cost_usd": 35.0,
                }
            ],
            "cots_parts": [],
            "environment_drill_operations": [],
            "final_assembly": [
                {
                    "name": "qty_probe",
                    "config": {
                        "dofs": [],
                    },
                }
            ],
            "totals": {
                "estimated_unit_cost_usd": 35.0,
                "estimated_weight_g": 2.7,
                "estimate_confidence": "high",
            },
        }
        custom_config = {
            "benchmark_operations": {
                "drilling": {
                    "cost_per_hole_usd": 2.0,
                }
            }
        }
        script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    part = Box(10, 10, 10)
    part = part.move(Location((0, 0, 5)))
    part.label = "qty_probe"
    part.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum_6061",
    )
    return part
"""

        await setup_workspace(
            client,
            base_headers,
            {
                "plan.md": valid_plan,
                "todo.md": valid_todo,
                "benchmark_definition.yaml": objectives,
                "assembly_definition.yaml": assembly_definition,
                "manufacturing_config.yaml": custom_config,
                "script.py": script,
            },
        )

        exec_resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=(
                    "python "
                    "/home/maksym/Work/proj/Problemologist/Problemologist-AI/"
                    "skills/manufacturing-knowledge/scripts/validate_and_price.py"
                ),
                timeout=60,
            ).model_dump(mode="json"),
            headers=base_headers,
        )
        assert exec_resp.status_code == 200, exec_resp.text
        exec_data = ExecuteResponse.model_validate(exec_resp.json())
        assert exec_data.exit_code == 0, exec_data.stderr

        submit_req = BenchmarkToolRequest(
            script_path="script.py",
            reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success, simulate_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())

    assert submit_data.success is False
    assert "Unit cost at requested quantity 1" in submit_data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_019_single_part_benchmark_submit_succeeds_without_cost_gate(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
):
    """INT-019: benchmark submit should ignore objective cost caps even with drilling."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        objectives = valid_objectives.model_copy(deep=True)
        objectives.constraints.estimated_solution_cost_usd = 500.0
        objectives.constraints.estimated_solution_weight_g = 10000.0
        objectives.benchmark_parts = [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "allows_engineer_interaction": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["fastener"],
                        "drill_policy": {
                            "allowed": True,
                            "max_hole_count": 1,
                            "diameter_range_mm": [3.0, 5.0],
                            "max_depth_mm": 10.0,
                        },
                    },
                },
            }
        ]

        assembly_definition = valid_cost.model_copy(deep=True)
        assembly_definition.constraints.benchmark_max_unit_cost_usd = 2000.0
        assembly_definition.constraints.planner_target_max_unit_cost_usd = 1500.0
        assembly_definition.environment_drill_operations = [
            {
                "target_part_id": "environment_fixture",
                "hole_id": "mount_left",
                "diameter_mm": 4.0,
                "depth_mm": 8.0,
                "quantity": 1,
            }
        ]
        assembly_definition.totals.estimated_unit_cost_usd = 1000.0
        custom_config = {
            "benchmark_operations": {
                "drilling": {
                    "cost_per_hole_usd": 1000.0,
                }
            }
        }

        goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

        await setup_workspace(
            client,
            base_headers,
            {
                "plan.md": valid_plan,
                "todo.md": valid_todo,
                "benchmark_definition.yaml": objectives,
                "benchmark_assembly_definition.yaml": assembly_definition,
                "manufacturing_config.yaml": custom_config,
                "script.py": goal_script,
            },
        )

        submit_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success, submit_data.message
        assert submit_data.artifacts is not None
        benchmark_manifest = submit_data.artifacts.review_manifests_json.get(
            ".manifests/benchmark_review_manifest.json"
        )
        assert benchmark_manifest is not None
        parsed_manifest = json.loads(benchmark_manifest)
        assert (
            parsed_manifest.get("reviewer_stage") == AgentName.BENCHMARK_REVIEWER.value
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_010_submit_handoff_accepts_cheaper_workspace_drilling_override(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
):
    """INT-010: handoff validation must respect cheaper workspace drilling overrides."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        objectives = valid_objectives.model_copy(deep=True)
        objectives.constraints.estimated_solution_cost_usd = 500.0
        objectives.constraints.estimated_solution_weight_g = 10000.0
        objectives.benchmark_parts = [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "allows_engineer_interaction": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["fastener"],
                        "drill_policy": {
                            "allowed": True,
                            "max_hole_count": 1,
                            "diameter_range_mm": [3.0, 5.0],
                            "max_depth_mm": 10.0,
                        },
                    },
                },
            }
        ]

        assembly_definition = valid_cost.model_copy(deep=True)
        assembly_definition.constraints.benchmark_max_unit_cost_usd = 50.0
        assembly_definition.constraints.planner_target_max_unit_cost_usd = 10.0
        assembly_definition.environment_drill_operations = [
            {
                "target_part_id": "environment_fixture",
                "hole_id": "mount_left",
                "diameter_mm": 4.0,
                "depth_mm": 8.0,
                "quantity": 1,
            }
        ]
        assembly_definition.totals.estimated_unit_cost_usd = 1.0
        custom_config = {
            "benchmark_operations": {
                "drilling": {
                    "cost_per_hole_usd": 1.0,
                }
            }
        }

        goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

        await setup_workspace(
            client,
            base_headers,
            {
                "plan.md": valid_plan,
                "todo.md": valid_todo,
                "benchmark_definition.yaml": objectives,
                "benchmark_assembly_definition.yaml": assembly_definition,
                "manufacturing_config.yaml": custom_config,
                "script.py": goal_script,
            },
        )

        submit_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success, submit_data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "prior_validation_missing",
        "prior_validation_stale_for_script",
        "prior_simulation_missing",
        "goal_not_reached_in_simulation",
    ]
)
@pytest.mark.asyncio
async def test_int_018_validate_and_price_integration_gate(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
    minimal_script,
):
    """INT-018: Verify submit_for_review gate requires validation + simulation on latest revision."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        relaxed_objectives = valid_objectives.model_copy(deep=True)
        relaxed_objectives.constraints.estimated_solution_cost_usd = 500.0
        relaxed_objectives.constraints.estimated_solution_weight_g = 10000.0

        goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "benchmark_definition.yaml": relaxed_objectives,
            "benchmark_assembly_definition.yaml": valid_cost,
            "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
            "script.py": goal_script,
        }
        await setup_workspace(client, base_headers, files)

        # 1) Missing validation gate must fail closed.
        delete_req = DeleteFileRequest(path="validation_results.json")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_req.model_dump(mode="json"),
            headers=base_headers,
        )

        submit_req = BenchmarkToolRequest(
            script_path="script.py", reviewer_stage=AgentName.BENCHMARK_REVIEWER
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "validation" in data.message.lower()

        # 2) Validation present but simulation missing must fail closed.
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        delete_sim_req = DeleteFileRequest(path="simulation_result.json")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_sim_req.model_dump(mode="json"),
            headers=base_headers,
        )
        missing_sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        missing_sim_data = BenchmarkToolResponse.model_validate(missing_sim_resp.json())
        assert not missing_sim_data.success
        assert "simulation" in missing_sim_data.message.lower()

        # 3) Latest revision enforcement must fail when script changed after gates.
        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message

        stale_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((0, 0, 5))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""
        await setup_workspace(client, base_headers, {"script.py": stale_script})
        stale_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        stale_data = BenchmarkToolResponse.model_validate(stale_resp.json())
        assert not stale_data.success
        assert (
            "stale" in stale_data.message.lower()
            or "re-run validate" in stale_data.message.lower()
            or "re-run simulate" in stale_data.message.lower()
        )

        # 4) Happy path: validate + simulate + submit on latest revision succeeds.
        await setup_workspace(client, base_headers, {"script.py": goal_script})
        val_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        assert val_resp.status_code == 200, val_resp.text
        val_data = BenchmarkToolResponse.model_validate(val_resp.json())
        assert val_data.success, val_data.message

        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=1000.0,
        )
        assert sim_resp.status_code == 200, sim_resp.text
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success, sim_data.message
        assert (
            "goal achieved" in sim_data.message.lower()
            or "goal zone" in sim_data.message.lower()
            or "green zone" in sim_data.message.lower()
        ), sim_data.message

        ok_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        ok_data = BenchmarkToolResponse.model_validate(ok_resp.json())
        assert ok_data.success, ok_data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(regexes=["simulation_failed"])
@pytest.mark.asyncio
async def test_int_018_benchmark_submit_accepts_yaml_motion_without_literal_tokens():
    """
    INT-018: benchmark submit must accept semantically valid motion facts from
    YAML even when plan.md/todo.md do not repeat the exact motion spellings.
    """
    session_id = f"INT-018-YAML-MOTION-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    plan_md = """## Learning Objective
Validate a benchmark handoff using structured motion facts from YAML and
equivalent natural-language motion descriptions.

## Geometry
- A small bridge-like scene with one laterally shifting benchmark fixture.
- The fixture uses a steady side-to-side trim at roughly 1.5 mm/s, bounded to
  a narrow correction window, with its motion details carried in the benchmark
  YAML artifacts.

## Objectives
- Place the target object in the goal zone while avoiding the forbid zone.
"""
    todo_md = """# TODO

- [x] Write the benchmark fixture YAML
- [x] Keep prose descriptive instead of literal
- [x] Document the constant drive and correction window in plain language
"""

    objectives = BenchmarkDefinition(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10.0, 10.0, 0.0), max=(20.0, 20.0, 20.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-50.0, -50.0, 0.0), max=(50.0, 50.0, 90.0)),
        ),
        benchmark_parts=[
            {
                "part_id": "bridge_gate",
                "label": "bridge_gate",
                "metadata": {
                    "material_id": "aluminum_6061",
                    "fixed": False,
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
        constraints=Constraints(
            estimated_solution_cost_usd=33.333333333333336,
            estimated_solution_weight_g=666.6666666666666,
        ),
    )
    assembly_definition = AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=50.0,
            benchmark_max_weight_g=1000.0,
            planner_target_max_unit_cost_usd=45.0,
            planner_target_max_weight_g=900.0,
        ),
        final_assembly=[
            PartConfig(
                name="bridge_gate",
                config=AssemblyPartConfig(
                    dofs=["slide_y"],
                    control=None,
                ),
            )
        ],
        totals=CostTotals(
            estimated_unit_cost_usd=0.0,
            estimated_weight_g=0.0,
            estimate_confidence="high",
        ),
    )

    goal_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1).translate((15, 15, 15))
    p.label = "ball"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""

    async with httpx.AsyncClient(timeout=300.0) as client:
        await setup_workspace(
            client,
            headers,
            {
                "plan.md": plan_md,
                "todo.md": todo_md,
                "benchmark_definition.yaml": objectives,
                "benchmark_assembly_definition.yaml": assembly_definition,
                "manufacturing_config.yaml": REPO_MANUFACTURING_CONFIG,
                "script.py": goal_script,
            },
        )

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(
                script_path="script.py",
                backend=SimulatorBackendType.MUJOCO,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="script.py",
                backend=SimulatorBackendType.MUJOCO,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success, simulate_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="script.py",
                reviewer_stage=AgentName.BENCHMARK_REVIEWER,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success, submit_data.message
        assert submit_data.artifacts is not None, submit_data
        assert submit_data.artifacts.render_paths, submit_data.artifacts
        assert all(
            not path.endswith(".yaml") for path in submit_data.artifacts.render_paths
        ), submit_data.artifacts.render_paths
        assert not any(
            path.endswith("benchmark_definition.yaml")
            or path.endswith("benchmark_assembly_definition.yaml")
            for path in submit_data.artifacts.render_paths
        ), submit_data.artifacts.render_paths
        assert not any(
            path.endswith("benchmark_definition.yaml")
            or path.endswith("benchmark_assembly_definition.yaml")
            for path in submit_data.artifacts.render_blobs_base64
        ), submit_data.artifacts.render_blobs_base64
