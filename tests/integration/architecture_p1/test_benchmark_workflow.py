import hashlib
import uuid
from pathlib import Path

import pytest
import yaml
from httpx import AsyncClient

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
    EpisodePhase,
    EpisodeStatus,
    ReviewDecision,
    TerminalReason,
    TraceType,
)
from shared.models.schemas import AssemblyDefinition, BenchmarkDefinition
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    PlanReviewManifest,
    RenderManifest,
    ReviewManifest,
)
from tests.integration.agent.helpers import (
    repo_git_revision,
    wait_for_benchmark_state,
)

# Adjust URL to your controller if different
CONTROLLER_URL = "http://127.0.0.1:18000"

pytestmark = pytest.mark.xdist_group(name="physics_sims")


def _asset_path(asset_path: str | Path) -> Path:
    return Path(str(asset_path).lstrip("/"))


async def _wait_for_planned_or_failed_episode(
    client: AsyncClient, episode_id: str
) -> EpisodeResponse:
    return EpisodeResponse.model_validate(
        await wait_for_benchmark_state(
            client,
            episode_id,
            timeout_s=120.0,
            terminal_statuses={EpisodeStatus.PLANNED, EpisodeStatus.FAILED},
        )
    )


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_benchmark_planner_cad_reviewer_path():
    """
    INT-031: Benchmark planner -> CAD -> reviewer path

    Verifies:
    1. Benchmark generation trigger
    2. Successful completion of the workflow
    3. Existence of required artifacts (plan.md, benchmark_definition.yaml, Reviews)
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        # 1. Trigger Benchmark Generation
        request = BenchmarkGenerateRequest(
            prompt="INT-005: Create a simple path planning benchmark with a wall and a goal.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to trigger benchmark: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = str(benchmark_resp.session_id)

        initial_episode = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                session_id,
                timeout_s=150.0,
                terminal_statuses={
                    EpisodeStatus.PLANNED,
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                    EpisodeStatus.CANCELLED,
                },
            )
        )
        if initial_episode.status == EpisodeStatus.PLANNED:
            # WP08: Call confirm to continue from planning to execution
            confirm_resp = await client.post(
                f"/benchmark/{session_id}/confirm",
                json=ConfirmRequest(comment="Looks good").model_dump(),
            )
            assert confirm_resp.status_code in [200, 202]
            final_episode = EpisodeResponse.model_validate(
                await wait_for_benchmark_state(
                    client,
                    session_id,
                    timeout_s=150.0,
                    terminal_statuses={
                        EpisodeStatus.COMPLETED,
                        EpisodeStatus.FAILED,
                        EpisodeStatus.CANCELLED,
                    },
                )
            )
        else:
            final_episode = initial_episode

        if final_episode.status == EpisodeStatus.FAILED:
            pytest.fail(
                f"Benchmark generation failed with status: {final_episode.status}"
            )

        final_metadata = final_episode.metadata_vars
        assert final_metadata is not None, "Episode metadata is missing."
        assert final_metadata.terminal_reason == TerminalReason.APPROVED
        assert final_metadata.failure_class is None
        assert final_metadata.episode_phase == EpisodePhase.BENCHMARK_REVIEWING

        # 3. Verify Artifacts from episode assets
        episode_resp = await client.get(f"/episodes/{session_id}")
        assert episode_resp.status_code == 200, (
            f"Failed to fetch episode assets: {episode_resp.text}"
        )
        episode_data = EpisodeResponse.model_validate(episode_resp.json())
        artifact_paths = [_asset_path(a.s3_path) for a in (episode_data.assets or [])]
        traces = episode_data.traces or []
        submit_plan_traces = [
            t
            for t in traces
            if t.trace_type.value == "TOOL_START" and t.name == "submit_plan"
        ]
        inspect_media_traces = [
            t
            for t in traces
            if t.trace_type.value == "TOOL_START" and t.name == "inspect_media"
        ]

        assert Path("plan.md") in artifact_paths, (
            f"plan.md missing. Artifacts: {artifact_paths}"
        )
        assert Path("benchmark_definition.yaml") in artifact_paths, (
            f"benchmark_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        assert Path("benchmark_script.py") in artifact_paths, (
            f"benchmark_script.py missing. Artifacts: {artifact_paths}"
        )
        assert Path(".manifests/current_role.json") in artifact_paths, (
            f"current_role.json missing. Artifacts: {artifact_paths}"
        )
        benchmark_definition_paths = [
            p for p in artifact_paths if p == Path("benchmark_definition.yaml")
        ]
        benchmark_definition_resp = await client.get(
            f"/episodes/{session_id}/assets/{benchmark_definition_paths[0]}"
        )
        assert benchmark_definition_resp.status_code == 200, (
            benchmark_definition_resp.text
        )
        benchmark_definition = BenchmarkDefinition.model_validate(
            yaml.safe_load(benchmark_definition_resp.text)
        )
        assert benchmark_definition.benchmark_parts, (
            "benchmark_definition.yaml must persist at least one benchmark_parts entry."
        )
        assert len(
            {part.part_id for part in benchmark_definition.benchmark_parts}
        ) == len(benchmark_definition.benchmark_parts), (
            "benchmark_definition.yaml must preserve unique benchmark_parts.part_id "
            "values."
        )
        assert len(
            {part.label for part in benchmark_definition.benchmark_parts}
        ) == len(benchmark_definition.benchmark_parts), (
            "benchmark_definition.yaml must preserve unique benchmark_parts.label "
            "values."
        )
        assert benchmark_definition.physics.fem_enabled is False
        assert benchmark_definition.fluids == []
        assert benchmark_definition.objectives.fluid_objectives == []
        assert benchmark_definition.objectives.stress_objectives == []
        assert benchmark_definition.electronics_requirements is None
        assert benchmark_definition.moved_object.material_id
        assert isinstance(
            benchmark_definition.benchmark_parts[
                0
            ].metadata.allows_engineer_interaction,
            bool,
        )
        assert (
            benchmark_definition.benchmark_parts[0].metadata.allows_engineer_interaction
            is False
        )
        assert benchmark_definition.randomization.runtime_jitter_enabled is True
        assert "randomization:" in benchmark_definition_resp.text
        assert "runtime_jitter:" in benchmark_definition_resp.text
        plan_paths = [p for p in artifact_paths if p == Path("plan.md")]
        plan_resp = await client.get(f"/episodes/{session_id}/assets/{plan_paths[0]}")
        assert plan_resp.status_code == 200, plan_resp.text
        assert Path("benchmark_assembly_definition.yaml") in artifact_paths, (
            f"benchmark_assembly_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        assembly_paths = [
            p for p in artifact_paths if p == Path("benchmark_assembly_definition.yaml")
        ]
        assembly_resp = await client.get(
            f"/episodes/{session_id}/assets/{assembly_paths[0]}"
        )
        assert assembly_resp.status_code == 200, assembly_resp.text
        benchmark_assembly_definition = AssemblyDefinition.model_validate(
            yaml.safe_load(assembly_resp.text)
        )
        assert benchmark_assembly_definition.manufactured_parts == []
        assert benchmark_assembly_definition.cots_parts == []
        assert benchmark_assembly_definition.final_assembly == []
        assert submit_plan_traces, (
            "Expected planner to call submit_plan before workflow completion."
        )
        assert inspect_media_traces, (
            "Expected benchmark reviewer to inspect a render before approval."
        )
        assert any(t.name == "media_inspection" for t in traces), (
            "media_inspection event missing from benchmark reviewer run."
        )
        assert any(t.name == "llm_media_attached" for t in traces), (
            "llm_media_attached event missing from benchmark reviewer run."
        )
        assert Path("validation_results.json") in artifact_paths, (
            f"validation_results.json missing. Artifacts: {artifact_paths}"
        )
        assert Path("simulation_result.json") in artifact_paths, (
            f"simulation_result.json missing. Artifacts: {artifact_paths}"
        )
        manifest_paths = [
            p
            for p in artifact_paths
            if p == Path(".manifests/benchmark_plan_review_manifest.json")
        ]
        assert manifest_paths, (
            f"benchmark_plan_review_manifest.json missing. Artifacts: {artifact_paths}"
        )
        assert Path("benchmark-plan-review-decision-round-1.yaml") in artifact_paths, (
            "benchmark plan review decision file missing from artifacts. "
            f"Artifacts: {artifact_paths}"
        )
        assert Path("benchmark-plan-review-comments-round-1.yaml") in artifact_paths, (
            "benchmark plan review comments file missing from artifacts. "
            f"Artifacts: {artifact_paths}"
        )
        manifest_paths = [
            p
            for p in artifact_paths
            if p == Path(".manifests/benchmark_review_manifest.json")
        ]
        assert manifest_paths, (
            f"benchmark_review_manifest.json missing. Artifacts: {artifact_paths}"
        )
        assert not any(
            p == Path(".manifests/engineering_execution_handoff_manifest.json")
            for p in artifact_paths
        ), (
            "Benchmark workflow must not emit "
            "'engineering_execution_handoff_manifest.json'. "
            f"Artifacts: {artifact_paths}"
        )
        assert any(p.parent.name == ".manifests" for p in manifest_paths), (
            f"review manifest must be in .manifests/. Found: {manifest_paths}"
        )
        manifest_resp = await client.get(
            f"/episodes/{session_id}/assets/{manifest_paths[0]}"
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = ReviewManifest.model_validate_json(manifest_resp.text)
        assert manifest.status == "ready_for_review"
        assert manifest.session_id == session_id
        assert manifest.episode_id == str(benchmark_resp.episode_id)
        assert manifest.worker_session_id == session_id
        assert manifest.revision == repo_git_revision()
        assert manifest.benchmark_episode_id == str(benchmark_resp.episode_id)
        assert manifest.benchmark_worker_session_id == session_id
        assert manifest.benchmark_revision == repo_git_revision()
        assert manifest.solution_revision == repo_git_revision()
        assert manifest.validation_success is True
        assert manifest.simulation_success is True
        assert manifest.goal_reached is True
        assert manifest.preview_evidence_paths
        assert set(manifest.preview_evidence_paths) == set(manifest.renders)

        benchmark_assembly_definition_path = next(
            p for p in artifact_paths if p == Path("benchmark_assembly_definition.yaml")
        )
        benchmark_assembly_definition_resp = await client.get(
            f"/episodes/{session_id}/assets/{benchmark_assembly_definition_path}"
        )
        assert benchmark_assembly_definition_resp.status_code == 200, (
            benchmark_assembly_definition_resp.text
        )
        benchmark_assembly_definition = yaml.safe_load(
            benchmark_assembly_definition_resp.text
        )
        assert manifest.environment_version == benchmark_assembly_definition["version"]

        script_resp = await client.get(
            f"/episodes/{session_id}/assets/{manifest.script_path}"
        )
        assert script_resp.status_code == 200, script_resp.text
        assert (
            hashlib.sha256(script_resp.text.encode("utf-8")).hexdigest()
            == manifest.script_sha256
        )


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_benchmark_plan_reviewer_rejects_unsolvable_render_bundle():
    """
    INT-203: A schema-valid benchmark that is logically unsolvable must be
    rejected explicitly by the benchmark plan reviewer and must not advance to
    benchmark coding.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        request = BenchmarkGenerateRequest(
            prompt="INT-203 benchmark planner solvability rejection.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [200, 202], resp.text
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = str(benchmark_resp.session_id)
        episode_id = str(benchmark_resp.episode_id)

        latest_episode = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                session_id,
                timeout_s=120.0,
                terminal_statuses=set(),
                predicate=lambda candidate: (
                    any(
                        trace.name == "review_decision"
                        and trace.metadata_vars is not None
                        and trace.metadata_vars.decision == ReviewDecision.REJECT_PLAN
                        and "UNSOLVABLE_SCENARIO" in (trace.content or "")
                        for trace in (candidate.traces or [])
                    )
                    and any(
                        _asset_path(asset.s3_path)
                        == Path("benchmark-plan-review-decision-round-1.yaml")
                        for asset in (candidate.assets or [])
                    )
                    and any(
                        _asset_path(asset.s3_path)
                        == Path("benchmark-plan-review-comments-round-1.yaml")
                        for asset in (candidate.assets or [])
                    )
                    and any(
                        _asset_path(asset.s3_path)
                        == Path(".manifests/benchmark_plan_review_manifest.json")
                        for asset in (candidate.assets or [])
                    )
                ),
            )
        )
        traces = latest_episode.traces or []
        rejected_traces = [
            trace
            for trace in traces
            if trace.name == "review_decision"
            and trace.metadata_vars is not None
            and trace.metadata_vars.decision == ReviewDecision.REJECT_PLAN
            and "UNSOLVABLE_SCENARIO" in (trace.content or "")
        ]
        rejected_review_trace = max(rejected_traces, key=lambda trace: trace.id)
        artifact_paths = [
            _asset_path(asset.s3_path) for asset in (latest_episode.assets or [])
        ]
        decision_paths = [
            path
            for path in artifact_paths
            if path == Path("benchmark-plan-review-decision-round-1.yaml")
        ]
        comments_paths = [
            path
            for path in artifact_paths
            if path == Path("benchmark-plan-review-comments-round-1.yaml")
        ]
        manifest_paths = [
            path
            for path in artifact_paths
            if path == Path(".manifests/benchmark_plan_review_manifest.json")
        ]

        assert rejected_review_trace is not None, (
            "Expected benchmark plan reviewer rejection mentioning UNSOLVABLE_SCENARIO."
        )
        assert not any(
            trace.name == "benchmark_coder"
            and "Starting task phase" in (trace.content or "")
            for trace in traces
        ), "Benchmark coder must not start after unsolvable benchmark rejection."

        artifact_paths = [
            _asset_path(asset.s3_path) for asset in (latest_episode.assets or [])
        ]
        assert decision_paths, f"Decision artifact missing. Artifacts: {artifact_paths}"
        assert comments_paths, f"Comments artifact missing. Artifacts: {artifact_paths}"
        assert manifest_paths, f"Manifest artifact missing. Artifacts: {artifact_paths}"

        comments_resp = await client.get(
            f"/episodes/{episode_id}/assets/{comments_paths[0]}"
        )
        assert comments_resp.status_code == 200, comments_resp.text
        comments = yaml.safe_load(comments_resp.text)
        assert comments["summary"].startswith("REJECT_PLAN:"), comments
        assert comments["checklist"]["render_count"] == 2
        assert comments["checklist"]["visual_inspection_satisfied"] is True
        assert comments["checklist"]["latest_revision_verified"] is True

        plan_review_manifest_paths = [
            path
            for path in artifact_paths
            if path == Path(".manifests/benchmark_plan_review_manifest.json")
        ]
        assert plan_review_manifest_paths, (
            f"benchmark_plan_review_manifest.json missing. Artifacts: {artifact_paths}"
        )
        manifest_resp = await client.get(
            f"/episodes/{session_id}/assets/{plan_review_manifest_paths[0]}"
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = PlanReviewManifest.model_validate_json(manifest_resp.text)
        assert manifest.status == "ready_for_review"
        assert manifest.reviewer_stage == AgentName.BENCHMARK_PLAN_REVIEWER
        assert manifest.planner_node_type == AgentName.BENCHMARK_PLANNER
        assert manifest.artifact_hashes, manifest
        assert {
            "plan.md",
            "todo.md",
            "benchmark_definition.yaml",
            "benchmark_assembly_definition.yaml",
        }.issubset(manifest.artifact_hashes), manifest
        assert comments["checklist"]["review_manifest_revision"], comments

        render_manifest_paths = [
            p for p in artifact_paths if p == Path("renders/render_manifest.json")
        ]
        assert render_manifest_paths, (
            f"renders/render_manifest.json missing. Artifacts: {artifact_paths}"
        )
        render_manifest_resp = await client.get(
            f"/episodes/{session_id}/assets/{render_manifest_paths[0]}"
        )
        assert render_manifest_resp.status_code == 200, render_manifest_resp.text
        render_manifest = RenderManifest.model_validate_json(render_manifest_resp.text)
        assert render_manifest.artifacts, render_manifest


@pytest.mark.integration_p1
@pytest.mark.allow_backend_errors(
    regexes=[
        r"planner_handoff_validation_failed",
        r"benchmark_refusal",
        r"CONTRADICTORY_CONSTRAINTS",
    ]
)
@pytest.mark.asyncio
async def test_int_200_benchmark_workflow_rejects_hidden_motion_handoff():
    """
    INT-200: A benchmark workflow seeded with hidden benchmark-side motion must
    fail closed as contradictory planner constraints before the plan reviewer or
    benchmark coder can start.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        req = AgentRunRequest(
            task="INT-200 benchmark workflow hidden motion contract.",
            session_id=f"INT-200-{uuid.uuid4().hex[:8]}",
            agent_name=AgentName.BENCHMARK_PLANNER,
            start_node=AgentName.BENCHMARK_PLANNER,
        )
        run_resp = await client.post("/api/agent/run", json=req.model_dump(mode="json"))
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _wait_for_planned_or_failed_episode(client, str(run.episode_id))
        metadata = episode.metadata_vars
        assert metadata is not None
        assert episode.status == EpisodeStatus.PLANNED
        assert metadata.terminal_reason == TerminalReason.HANDOFF_INVARIANT_VIOLATION
        assert metadata.failure_class == "AGENT_SEMANTIC_FAILURE"
        assert any(
            "CONTRADICTORY_CONSTRAINTS" in log
            and "bridge_reference_table" in log
            and "slide_y" in log
            for log in (metadata.validation_logs or [])
        ), metadata.validation_logs
        assert any(
            "bridge_reference_table" in log and "slide_y" in log
            for log in (metadata.validation_logs or [])
        ), metadata.validation_logs
        assert any(
            "planner_execution: missing structured planner output" in log
            for log in (metadata.validation_logs or [])
        ), metadata.validation_logs
        assert any(
            "control=mode=<MotorControlMode.ON_OFF: 'ON_OFF'>" in log
            for log in (metadata.validation_logs or [])
        ), metadata.validation_logs
        assert any("speed=0.15" in log for log in (metadata.validation_logs or [])), (
            metadata.validation_logs
        )

        traces = episode.traces or []
        submit_plan_traces = [
            trace
            for trace in traces
            if trace.trace_type == TraceType.TOOL_START and trace.name == "submit_plan"
        ]
        assert len(submit_plan_traces) >= 2, (
            "Benchmark workflow must retry submit_plan before fail-closed rejection."
        )
        assert not any(
            trace.name == "benchmark_plan_reviewer"
            and "Starting task phase" in (trace.content or "")
            for trace in traces
        ), "Benchmark plan reviewer should not start after hidden-motion rejection."
        assert not any(
            trace.name == "benchmark_coder"
            and "Starting task phase" in (trace.content or "")
            for trace in traces
        ), "Benchmark coder should not start after hidden-motion rejection."


@pytest.mark.integration_p1
@pytest.mark.allow_backend_errors(
    regexes=[
        r"planner_handoff_validation_failed",
        r"benchmark_refusal",
        r"UNSOLVABLE_SCENARIO",
    ]
)
@pytest.mark.asyncio
async def test_int_202_benchmark_workflow_rejects_unsupported_motion_handoff():
    """
    INT-202: A benchmark workflow seeded with unsupported benchmark motion must
    fail closed before the plan reviewer or benchmark coder can start.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        req = AgentRunRequest(
            task="INT-202 benchmark workflow unsupported motion contract.",
            session_id=f"INT-202-{uuid.uuid4().hex[:8]}",
            agent_name=AgentName.BENCHMARK_PLANNER,
            start_node=AgentName.BENCHMARK_PLANNER,
        )
        run_resp = await client.post("/api/agent/run", json=req.model_dump(mode="json"))
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _wait_for_planned_or_failed_episode(client, str(run.episode_id))
        metadata = episode.metadata_vars
        assert metadata is not None
        assert episode.status == EpisodeStatus.PLANNED
        assert metadata.terminal_reason == TerminalReason.HANDOFF_INVARIANT_VIOLATION
        assert metadata.failure_class == "AGENT_SEMANTIC_FAILURE"
        assert any(
            "UNSOLVABLE_SCENARIO" in log for log in (metadata.validation_logs or [])
        ), metadata.validation_logs
        assert any(
            "unsupported benchmark motion" in log.lower()
            for log in (metadata.validation_logs or [])
        ), metadata.validation_logs
        assert any(
            "bridge_trim_unsupported_v1" in log
            for log in (metadata.validation_logs or [])
        ), metadata.validation_logs

        traces = episode.traces or []
        submit_plan_traces = [
            trace
            for trace in traces
            if trace.trace_type == TraceType.TOOL_START and trace.name == "submit_plan"
        ]
        assert len(submit_plan_traces) >= 2, (
            "Benchmark workflow must retry submit_plan before fail-closed rejection."
        )
        assert not any(
            trace.name == "benchmark_plan_reviewer"
            and "Starting task phase" in (trace.content or "")
            for trace in traces
        ), (
            "Benchmark plan reviewer should not start after unsupported motion rejection."
        )
        assert not any(
            trace.name == "benchmark_coder"
            and "Starting task phase" in (trace.content or "")
            for trace in traces
        ), "Benchmark coder should not start after unsupported motion rejection."


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_benchmark_request_validation_rejects_invalid_objectives():
    """The benchmark API must reject invalid deterministic objective values."""
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        invalid_generate = await client.post(
            "/benchmark/generate",
            json={
                "prompt": "Create a benchmark",
                "max_cost": -1,
            },
        )
        assert invalid_generate.status_code == 422

        invalid_update = await client.post(
            f"/benchmark/{uuid.uuid4()}/objectives",
            json={
                "max_cost": 0,
                "max_weight": -5,
                "target_quantity": 0,
            },
        )
        assert invalid_update.status_code == 422
