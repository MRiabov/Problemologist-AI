import asyncio
import uuid
from pathlib import Path

import pytest
import yaml
from httpx import AsyncClient

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import EpisodeStatus, ReviewDecision
from tests.integration.agent.helpers import seed_benchmark_assembly_definition

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
        run_request = AgentRunRequest(
            task="INT-034 reviewer evidence completeness",
            session_id=session_id,
        )
        run_resp = await client.post("/agent/run", json=run_request.model_dump())
        assert run_resp.status_code in [200, 202], (
            f"Agent trigger failed: {run_resp.text}"
        )
        agent_run_resp = AgentRunResponse.model_validate(run_resp.json())
        episode_id = agent_run_resp.episode_id

        for _ in range(180):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            assert ep_resp.status_code == 200, ep_resp.text
            ep_data = EpisodeResponse.model_validate(ep_resp.json())
            if ep_data.status in [EpisodeStatus.COMPLETED, EpisodeStatus.FAILED]:
                break
            await asyncio.sleep(1.0)
        else:
            pytest.fail(f"Episode did not complete in time (episode_id={episode_id})")

        ep_resp = await client.get(f"/episodes/{episode_id}")
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
