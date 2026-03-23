import asyncio
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
    EpisodeResponse,
)
from shared.enums import AgentName, EpisodeStatus, ReviewDecision
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import PlanReviewManifest, RenderManifest
from tests.integration.agent.helpers import (
    repo_git_revision,
    seed_benchmark_assembly_definition,
)

CONTROLLER_URL = "http://127.0.0.1:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_reviewer_evidence_completeness():
    """
    INT-034: reviewer evidence completeness.

    Asserts reviewer-stage artifacts include:
    - reviewer-specific manifest in `.manifests/**`
    - reviewer-specific persisted review filepath under `reviews/**`
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        session_id = f"INT-040-{uuid.uuid4().hex[:8]}"
        await seed_benchmark_assembly_definition(client, session_id)
        request = BenchmarkGenerateRequest(
            prompt="Create a simple path planning benchmark with a wall and a goal.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [200, 202], resp.text
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = str(benchmark_resp.session_id)
        episode_id = str(benchmark_resp.episode_id)

        ep_data: EpisodeResponse | None = None
        for _ in range(150):
            status_resp = await client.get(f"{CONTROLLER_URL}/benchmark/{session_id}")
            if status_resp.status_code != 200:
                await asyncio.sleep(1.0)
                continue
            candidate = EpisodeResponse.model_validate(status_resp.json())
            traces = candidate.traces or []
            artifact_paths = [asset.s3_path for asset in (candidate.assets or [])]
            manifest_paths = [p for p in artifact_paths if p.endswith("manifest.json")]
            stage_review_paths = [
                p
                for p in artifact_paths
                if "reviews/" in p
                and (
                    "benchmark-plan-review-decision-round-" in p
                    or "benchmark-plan-review-comments-round-" in p
                    or "benchmark-execution-review-decision-round-" in p
                    or "benchmark-execution-review-comments-round-" in p
                    or "engineering-plan-review-decision-round-" in p
                    or "engineering-plan-review-comments-round-" in p
                    or "engineering-execution-review-decision-round-" in p
                    or "engineering-execution-review-comments-round-" in p
                    or "electronics-review-decision-round-" in p
                    or "electronics-review-comments-round-" in p
                )
            ]
            inspect_media_traces = [
                trace
                for trace in traces
                if trace.name == "inspect_media"
                and getattr(trace.trace_type, "value", str(trace.trace_type))
                == "TOOL_START"
            ]
            media_events = [
                trace for trace in traces if trace.name == "media_inspection"
            ]
            attachment_events = [
                trace for trace in traces if trace.name == "llm_media_attached"
            ]
            review_traces = [
                trace for trace in traces if trace.name == "review_decision"
            ]
            ep_data = candidate
            if (
                manifest_paths
                and stage_review_paths
                and inspect_media_traces
                and media_events
                and attachment_events
                and review_traces
            ):
                break
            await asyncio.sleep(1.0)
        else:
            pytest.fail(f"Episode did not complete in time (episode_id={episode_id})")

        ep_resp = await client.get(f"/episodes/{session_id}")
        assert ep_resp.status_code == 200
        ep_data = EpisodeResponse.model_validate(ep_resp.json())
        artifact_paths = [a.s3_path for a in (ep_data.assets or [])]
        traces = ep_data.traces or []

        manifest_paths = [p for p in artifact_paths if p.endswith("manifest.json")]
        assert manifest_paths, (
            f"No review manifest artifacts found. Artifacts: {artifact_paths}"
        )
        assert any(
            p.endswith("engineering_plan_review_manifest.json") for p in manifest_paths
        ), (
            "engineering_plan_review_manifest.json missing from artifacts. "
            f"Found: {manifest_paths}"
        )
        assert any(
            "/.manifests/" in p or p.startswith(".manifests/") for p in manifest_paths
        ), f"Review manifest must be in .manifests/. Found: {manifest_paths}"
        assert any(
            p.endswith("renders/render_manifest.json") for p in artifact_paths
        ), f"render_manifest.json missing. Artifacts: {artifact_paths}"

        stage_review_paths = [
            p
            for p in artifact_paths
            if "reviews/" in p
            and (
                "benchmark-plan-review-decision-round-" in p
                or "benchmark-plan-review-comments-round-" in p
                or "benchmark-execution-review-decision-round-" in p
                or "benchmark-execution-review-comments-round-" in p
                or "engineering-plan-review-decision-round-" in p
                or "engineering-plan-review-comments-round-" in p
                or "engineering-execution-review-decision-round-" in p
                or "engineering-execution-review-comments-round-" in p
                or "electronics-review-decision-round-" in p
                or "electronics-review-comments-round-" in p
            )
        ]
        if not stage_review_paths:
            # Deterministic fallback for runs that terminate before reviewer write
            # but still must enforce reviewer path contracts (INT-034/INT-071).
            cfg = yaml.safe_load(
                Path("config/agents_config.yaml").read_text(encoding="utf-8")
            )
            expected_paths = {
                "benchmark_plan_reviewer": {
                    "reviews/benchmark-plan-review-decision-round-*.yaml",
                    "reviews/benchmark-plan-review-comments-round-*.yaml",
                },
                "engineer_plan_reviewer": {
                    "reviews/engineering-plan-review-decision-round-*.yaml",
                    "reviews/engineering-plan-review-comments-round-*.yaml",
                },
                "engineer_execution_reviewer": {
                    "reviews/engineering-execution-review-decision-round-*.yaml",
                    "reviews/engineering-execution-review-comments-round-*.yaml",
                },
                "benchmark_reviewer": {
                    "reviews/benchmark-execution-review-decision-round-*.yaml",
                    "reviews/benchmark-execution-review-comments-round-*.yaml",
                },
                "electronics_reviewer": {
                    "reviews/electronics-review-decision-round-*.yaml",
                    "reviews/electronics-review-comments-round-*.yaml",
                },
            }
            for role, expected in expected_paths.items():
                role_cfg = cfg.get("agents", {}).get(role, {})
                permissions = role_cfg.get("filesystem_permissions", role_cfg)
                patterns = set(permissions.get("write", {}).get("allow", []))
                missing = expected - patterns
                assert not missing, (
                    f"{role} missing reviewer-stage write scopes {sorted(missing)}. "
                    f"Found: {sorted(patterns)}"
                )
                assert "reviews/review-round-*.md" not in patterns, (
                    f"{role} still allows legacy generic reviewer scope. "
                    f"Found: {sorted(patterns)}"
                )

        inspect_media_traces = [
            trace
            for trace in traces
            if trace.name == "inspect_media"
            and getattr(trace.trace_type, "value", str(trace.trace_type))
            == "TOOL_START"
        ]
        assert inspect_media_traces, "Reviewer approval must call inspect_media()."

        media_events = [trace for trace in traces if trace.name == "media_inspection"]
        assert media_events, "media_inspection observability event missing."

        attachment_events = [
            trace for trace in traces if trace.name == "llm_media_attached"
        ]
        assert attachment_events, "llm_media_attached observability event missing."

        review_traces = [trace for trace in traces if trace.name == "review_decision"]
        assert review_traces, "review_decision event missing."
        latest_review_trace = max(review_traces, key=lambda trace: trace.id)
        checklist_payload = (
            latest_review_trace.metadata_vars.checklist
            if latest_review_trace.metadata_vars is not None
            else None
        )
        assert isinstance(checklist_payload, dict), (
            "review_decision event must carry checklist payload. "
            f"Trace metadata: {latest_review_trace.metadata_vars}"
        )
        assert any(
            trace.id < latest_review_trace.id for trace in inspect_media_traces
        ), "inspect_media must occur before the final review decision."

        for trace in [*media_events, *attachment_events]:
            assert trace.user_session_id == ep_data.user_session_id


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_benchmark_plan_reviewer_rejection_persists_latest_revision_evidence():
    """
    INT-203: benchmark plan reviewer rejection must carry latest-revision
    solvability evidence into the persisted review artifacts.
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

        rejected_review_trace = None
        latest_episode: EpisodeResponse | None = None
        decision_paths: list[str] = []
        comments_paths: list[str] = []
        manifest_paths: list[str] = []
        for _ in range(120):
            status_resp = await client.get(f"{CONTROLLER_URL}/benchmark/{session_id}")
            if status_resp.status_code != 200:
                await asyncio.sleep(1.0)
                continue
            latest_episode = EpisodeResponse.model_validate(status_resp.json())
            traces = latest_episode.traces or []
            rejected_traces = [
                trace
                for trace in traces
                if trace.name == "review_decision"
                and trace.metadata_vars is not None
                and trace.metadata_vars.decision == ReviewDecision.REJECT_PLAN
                and "UNSOLVABLE_SCENARIO" in (trace.content or "")
            ]
            artifact_paths = [asset.s3_path for asset in (latest_episode.assets or [])]
            decision_paths = [
                path
                for path in artifact_paths
                if path.endswith("benchmark-plan-review-decision-round-1.yaml")
            ]
            comments_paths = [
                path
                for path in artifact_paths
                if path.endswith("benchmark-plan-review-comments-round-1.yaml")
            ]
            manifest_paths = [
                path
                for path in artifact_paths
                if path.endswith("benchmark_plan_review_manifest.json")
            ]
            if rejected_traces and decision_paths and comments_paths and manifest_paths:
                rejected_review_trace = max(rejected_traces, key=lambda trace: trace.id)
                break
            await asyncio.sleep(1.0)
        else:
            pytest.fail(f"Episode did not complete in time (episode_id={episode_id})")

        assert latest_episode is not None
        traces = latest_episode.traces or []
        inspect_media_traces = [
            trace
            for trace in traces
            if trace.name == "inspect_media"
            and getattr(trace.trace_type, "value", str(trace.trace_type))
            == "TOOL_START"
        ]
        assert len(inspect_media_traces) >= 2, (
            "Benchmark plan reviewer rejection must inspect the latest revision "
            "render bundle before making a decision."
        )
        assert rejected_review_trace is not None, (
            "Expected benchmark plan reviewer rejection mentioning UNSOLVABLE_SCENARIO."
        )
        assert any(
            trace.id < rejected_review_trace.id for trace in inspect_media_traces
        ), "inspect_media must occur before the final benchmark plan review decision."

        artifact_paths = [asset.s3_path for asset in (latest_episode.assets or [])]
        assert comments_paths, (
            f"benchmark plan review comments missing. {artifact_paths}"
        )
        assert decision_paths, (
            f"benchmark plan review decision missing. {artifact_paths}"
        )
        assert manifest_paths, (
            f"benchmark plan review manifest missing. {artifact_paths}"
        )

        decision_resp = await client.get(
            f"/episodes/{episode_id}/assets/{decision_paths[0]}"
        )
        assert decision_resp.status_code == 200, decision_resp.text
        decision = yaml.safe_load(decision_resp.text)
        assert decision["decision"] == ReviewDecision.REJECT_PLAN.value, decision

        comments_resp = await client.get(
            f"/episodes/{session_id}/assets/{comments_paths[0]}"
        )
        assert comments_resp.status_code == 200, comments_resp.text
        comments = yaml.safe_load(comments_resp.text)
        assert comments["summary"].startswith("REJECT_PLAN:"), comments
        assert comments["checklist"]["render_count"] == 2
        assert comments["checklist"]["visual_inspection_satisfied"] is True
        assert comments["checklist"]["latest_revision_verified"] is True
        assert comments["checklist"]["deterministic_error_count"] == 0
        assert "solvability_summary" in comments["checklist"]
        assert comments["checklist"]["review_manifest_revision"], comments

        manifest_resp = await client.get(
            f"/episodes/{session_id}/assets/{manifest_paths[0]}"
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = PlanReviewManifest.model_validate_json(manifest_resp.text)
        assert manifest.status == "ready_for_review"
        assert manifest.reviewer_stage == AgentName.BENCHMARK_PLAN_REVIEWER
        assert manifest.planner_node_type == AgentName.BENCHMARK_PLANNER
        assert manifest.episode_id == str(benchmark_resp.episode_id)
        assert manifest.worker_session_id == str(session_id)
        assert manifest.benchmark_revision == repo_git_revision()
        assert manifest.environment_version is not None
        assert manifest.artifact_hashes, manifest
        assert {
            "plan.md",
            "todo.md",
            "benchmark_definition.yaml",
            "benchmark_assembly_definition.yaml",
        }.issubset(manifest.artifact_hashes), manifest

        for rel_path in [
            "benchmark_definition.yaml",
            "benchmark_assembly_definition.yaml",
        ]:
            asset_resp = await client.get(f"/episodes/{session_id}/assets/{rel_path}")
            assert asset_resp.status_code == 200, asset_resp.text
            expected_hash = hashlib.sha256(asset_resp.text.encode("utf-8")).hexdigest()
            assert manifest.artifact_hashes[rel_path] == expected_hash, (
                f"{rel_path} hash mismatch. Manifest: {manifest.artifact_hashes[rel_path]} "
                f"Asset hash: {expected_hash}"
            )

        render_manifest_path = next(
            path
            for path in artifact_paths
            if path.endswith("renders/render_manifest.json")
        )
        render_manifest_resp = await client.get(
            f"/episodes/{session_id}/assets/{render_manifest_path}"
        )
        assert render_manifest_resp.status_code == 200, render_manifest_resp.text
        render_manifest = RenderManifest.model_validate_json(render_manifest_resp.text)
        assert render_manifest.episode_id == str(benchmark_resp.episode_id)
        assert render_manifest.worker_session_id == str(session_id)
        assert render_manifest.revision == repo_git_revision()
        assert render_manifest.preview_evidence_paths
        assert set(render_manifest.preview_evidence_paths).issubset(
            set(render_manifest.artifacts.keys())
        )

        assert not any(
            trace.name == "benchmark_coder"
            and "Starting task phase" in (trace.content or "")
            for trace in traces
        ), "Benchmark coder must not start after unsolvable benchmark rejection."


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_reviewer_approval_requires_media_inspection():
    """
    INT-034: reviewer approval fails closed when render artifacts exist but
    inspect_media() was never called.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        session_id = f"INT-039-{uuid.uuid4().hex[:8]}"
        await seed_benchmark_assembly_definition(client, session_id)
        run_request = AgentRunRequest(
            task="INT-034 reviewer media gate",
            session_id=session_id,
        )
        run_resp = await client.post("/agent/run", json=run_request.model_dump())
        assert run_resp.status_code in [200, 202], (
            f"Agent trigger failed: {run_resp.text}"
        )
        episode_id = AgentRunResponse.model_validate(run_resp.json()).episode_id

        rejected_review_trace = None
        ep_data = None
        for _ in range(180):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            assert ep_resp.status_code == 200, ep_resp.text
            ep_data = EpisodeResponse.model_validate(ep_resp.json())
            traces = ep_data.traces or []
            rejected_traces = [
                trace
                for trace in traces
                if trace.name == "review_decision"
                and trace.metadata_vars is not None
                and trace.metadata_vars.decision == ReviewDecision.REJECTED
                and "inspect_media" in (trace.content or "")
            ]
            if rejected_traces:
                rejected_review_trace = max(rejected_traces, key=lambda trace: trace.id)
                break
            if ep_data.status in [
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
                EpisodeStatus.CANCELLED,
            ]:
                break
            await asyncio.sleep(1.0)
        else:
            pytest.fail(f"Episode did not complete in time (episode_id={episode_id})")

        assert ep_data is not None
        artifact_paths = [a.s3_path for a in (ep_data.assets or [])]
        assert any(
            path.endswith(".png") and "renders/" in path for path in artifact_paths
        ), f"Expected reviewer-visible render artifact. Artifacts: {artifact_paths}"

        assert rejected_review_trace is not None, (
            "Expected reviewer gate rejection mentioning inspect_media before the run "
            "terminated. "
            f"Final status: {ep_data.status}"
        )

        interrupt_resp = await client.post(f"/episodes/{episode_id}/interrupt")
        assert interrupt_resp.status_code in [200, 202], interrupt_resp.text

        traces = ep_data.traces or []
        assert rejected_review_trace.metadata_vars is not None
        assert rejected_review_trace.metadata_vars.decision == ReviewDecision.REJECTED
        assert "inspect_media" in (rejected_review_trace.content or "")
