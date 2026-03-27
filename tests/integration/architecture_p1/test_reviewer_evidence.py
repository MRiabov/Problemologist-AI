import ast
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
    seed_current_revision_render_preview,
    seed_execution_reviewer_handover,
    wait_for_benchmark_state,
    wait_for_episode_state,
)

CONTROLLER_URL = "http://127.0.0.1:18000"


def _has_review_artifacts(
    episode: EpisodeResponse, *, required_checklist_pairs: tuple[tuple[str, str], ...]
) -> bool:
    traces = episode.traces or []
    review_traces = [
        trace
        for trace in traces
        if trace.name == "review_decision" and trace.metadata_vars is not None
    ]
    return all(
        any(
            trace.metadata_vars.checklist.get(checklist_key) == expected_value
            for trace in review_traces
        )
        for checklist_key, expected_value in required_checklist_pairs
    )


def _parse_event_payload(trace) -> dict:
    if not trace.content:
        return {}
    try:
        payload = ast.literal_eval(trace.content)
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _is_inspect_media_trace(trace) -> bool:
    return (
        trace.name in {"inspect_media", "inspect_media_tool"}
        and getattr(trace.trace_type, "value", str(trace.trace_type)) == "TOOL_START"
    )


def _benchmark_plan_review_artifacts_ready(episode: EpisodeResponse) -> bool:
    traces = episode.traces or []
    rejected_traces = [
        trace
        for trace in traces
        if trace.name == "review_decision"
        and trace.metadata_vars is not None
        and trace.metadata_vars.decision == ReviewDecision.REJECT_PLAN
        and "UNSOLVABLE_SCENARIO" in (trace.content or "")
    ]
    artifact_paths = [asset.s3_path for asset in (episode.assets or [])]
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
    return bool(
        rejected_traces and decision_paths and comments_paths and manifest_paths
    )


async def _wait_for_review_evidence(
    client: AsyncClient,
    *,
    episode_id: str,
    required_checklist_pairs: tuple[tuple[str, str], ...],
    attempts: int = 15,
) -> EpisodeResponse:
    return EpisodeResponse.model_validate(
        await wait_for_episode_state(
            client,
            episode_id,
            timeout_s=float(attempts),
            predicate=lambda episode: _has_review_artifacts(
                episode,
                required_checklist_pairs=required_checklist_pairs,
            ),
        )
    )


async def _wait_for_trace_name(
    client: AsyncClient,
    *,
    episode_id: str,
    trace_name: str,
    predicate,
    attempts: int = 15,
) -> list:
    episode = EpisodeResponse.model_validate(
        await wait_for_episode_state(
            client,
            episode_id,
            timeout_s=float(attempts),
            predicate=lambda episode: any(
                trace.name == trace_name and predicate(trace)
                for trace in (episode.traces or [])
            ),
        )
    )
    return [
        trace
        for trace in (episode.traces or [])
        if trace.name == trace_name and predicate(trace)
    ]


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

        ep_data = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                session_id,
                timeout_s=180.0,
                terminal_statuses=set(),
                predicate=lambda candidate: (
                    any(
                        p.endswith("manifest.json")
                        for p in [asset.s3_path for asset in (candidate.assets or [])]
                    )
                    and any(
                        "reviews/" in p
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
                        for p in [asset.s3_path for asset in (candidate.assets or [])]
                    )
                    and any(
                        _is_inspect_media_trace(trace)
                        for trace in (candidate.traces or [])
                    )
                    and any(
                        trace.name == "media_inspection"
                        for trace in (candidate.traces or [])
                    )
                    and any(
                        trace.name == "llm_media_attached"
                        for trace in (candidate.traces or [])
                    )
                    and any(
                        trace.name == "review_decision"
                        for trace in (candidate.traces or [])
                    )
                ),
            )
        )

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
            p.endswith("benchmark_plan_review_manifest.json") for p in manifest_paths
        ), (
            "benchmark_plan_review_manifest.json missing from artifacts. "
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
            trace for trace in traces if _is_inspect_media_trace(trace)
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
async def test_engineering_dof_review_evidence_uses_canonical_keys():
    """
    INT-074/INT-075: canonical DOF checklist keys must persist through both
    the plan-review minimality path and the execution-review justification path.
    """

    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        session_id = f"INT-075-{uuid.uuid4().hex[:8]}"
        await seed_benchmark_assembly_definition(
            client,
            session_id,
            benchmark_max_unit_cost_usd=250.0,
            benchmark_max_weight_g=2500.0,
            planner_target_max_unit_cost_usd=250.0,
            planner_target_max_weight_g=2500.0,
        )
        await seed_execution_reviewer_handover(
            client,
            session_id=session_id,
            int_id="INT-075",
        )
        over_actuated_benchmark_definition = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_planner/entry_01/04__benchmark_definition.yaml"
        ).read_text(encoding="utf-8")
        over_actuated_plan = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_planner/entry_01/01__plan.md"
        ).read_text(encoding="utf-8")
        over_actuated_todo = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_planner/entry_01/02__todo.md"
        ).read_text(encoding="utf-8")
        over_actuated_assembly = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_coder/entry_01/01__assembly_definition.yaml"
        ).read_text(encoding="utf-8")
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "benchmark_definition.yaml",
                "content": over_actuated_benchmark_definition,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "plan.md",
                "content": over_actuated_plan,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "todo.md",
                "content": over_actuated_todo,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "assembly_definition.yaml",
                "content": over_actuated_assembly,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        request = AgentRunRequest(
            task="INT-074 canonical DOF review evidence gate",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_CODER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=request.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = EpisodeResponse.model_validate(
            await wait_for_episode_state(
                client,
                str(run.episode_id),
                timeout_s=180.0,
                terminal_statuses={
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                    EpisodeStatus.CANCELLED,
                    EpisodeStatus.PLANNED,
                },
                predicate=lambda episode: _has_review_artifacts(
                    episode,
                    required_checklist_pairs=(
                        ("dof_minimality", "pass"),
                        ("dof_deviation_justified", "pass"),
                    ),
                ),
            )
        )
        if not _has_review_artifacts(
            episode,
            required_checklist_pairs=(
                ("dof_minimality", "pass"),
                ("dof_deviation_justified", "pass"),
            ),
        ):
            episode = await _wait_for_review_evidence(
                client,
                episode_id=str(run.episode_id),
                required_checklist_pairs=(
                    ("dof_minimality", "pass"),
                    ("dof_deviation_justified", "pass"),
                ),
            )

        assert episode.status in {
            EpisodeStatus.COMPLETED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
            EpisodeStatus.PLANNED,
        } or _has_review_artifacts(
            episode,
            required_checklist_pairs=(
                ("dof_minimality", "pass"),
                ("dof_deviation_justified", "pass"),
            ),
        )

        plan_comments_path = "reviews/engineering-plan-review-comments-round-1.yaml"
        execution_comments_path = (
            "reviews/engineering-execution-review-comments-round-1.yaml"
        )

        plan_comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode.id}/assets/{plan_comments_path}"
        )
        assert plan_comments_resp.status_code == 200, plan_comments_resp.text
        plan_comments = yaml.safe_load(plan_comments_resp.text)
        assert plan_comments["checklist"]["dof_minimality"] == "pass", plan_comments

        execution_comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode.id}/assets/{execution_comments_path}"
        )
        assert execution_comments_resp.status_code == 200, execution_comments_resp.text
        execution_comments = yaml.safe_load(execution_comments_resp.text)
        assert execution_comments["checklist"]["dof_deviation_justified"] == "pass", (
            execution_comments
        )

        review_traces = [
            trace for trace in (episode.traces or []) if trace.name == "review_decision"
        ]
        execution_review_traces = [
            trace
            for trace in review_traces
            if trace.metadata_vars is not None
            and trace.metadata_vars.checklist.get("dof_deviation_justified") == "pass"
        ]
        assert execution_review_traces, review_traces
        plan_review_traces = [
            trace
            for trace in review_traces
            if trace.metadata_vars is not None
            and trace.metadata_vars.checklist.get("dof_minimality") == "pass"
        ]
        assert plan_review_traces, review_traces

        dof_event_traces = [
            trace
            for trace in (episode.traces or [])
            if trace.name == "excessive_dof_detected"
        ]
        assert dof_event_traces, "Expected excessive_dof_detected trace"
        dof_event_payloads = [_parse_event_payload(trace) for trace in dof_event_traces]
        plan_dof_payload = next(
            payload
            for payload in dof_event_payloads
            if payload.get("reviewer_stage") == "engineering_plan_reviewer"
        )
        execution_dof_payload = next(
            payload
            for payload in dof_event_payloads
            if payload.get("reviewer_stage") == "engineering_execution_reviewer"
        )
        for payload in (plan_dof_payload, execution_dof_payload):
            assert payload["expected_minimal_engineering_dofs"] == [
                "tx",
                "ty",
                "tz",
            ], payload
            assert (
                payload["expected_minimal_engineering_dofs"]
                == payload["expected_minimal_dofs"]
            ), payload


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_engineer_execution_reviewer_rejects_over_actuated_dofs_after_render_inspection():
    """
    INT-074: execution reviewer rejection path must inspect current-revision
    renders before rejecting on excessive DOFs.
    """

    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        session_id = f"INT-074-{uuid.uuid4().hex[:8]}"
        await seed_benchmark_assembly_definition(
            client,
            session_id,
            benchmark_max_unit_cost_usd=250.0,
            benchmark_max_weight_g=2500.0,
            planner_target_max_unit_cost_usd=250.0,
            planner_target_max_weight_g=2500.0,
        )
        # Use the INT-075 handoff pair so the reviewer entry check reaches the
        # over-actuated DOF rejection branch with a verifiable workspace state.
        await seed_execution_reviewer_handover(
            client,
            session_id=session_id,
            int_id="INT-075",
        )
        over_actuated_benchmark_definition = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_planner/entry_01/04__benchmark_definition.yaml"
        ).read_text(encoding="utf-8")
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "benchmark_definition.yaml",
                "content": over_actuated_benchmark_definition,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        over_actuated_todo = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_planner/entry_01/02__todo.md"
        ).read_text(encoding="utf-8")
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "todo.md",
                "content": over_actuated_todo,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        await seed_current_revision_render_preview(client, session_id=session_id)

        over_actuated_assembly = Path(
            "tests/integration/mock_responses/INT-075/"
            "engineer_coder/entry_01/01__assembly_definition.yaml"
        ).read_text(encoding="utf-8")
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "assembly_definition.yaml",
                "content": over_actuated_assembly,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": "plan.md",
                "content": (
                    "## 1. Solution Overview\n\n"
                    "INT-074 execution reviewer over-actuation deviation scenario.\n\n"
                    "## 2. Parts List\n\n"
                    "- planner_link\n\n"
                    "## 3. Assembly Strategy\n\n"
                    "1. Baseline assembly.\n\n"
                    "## 4. Cost & Weight Budget\n\n"
                    "- Within planner caps.\n\n"
                    "## 5. Risk Assessment\n\n"
                    "- Over-actuation drift risk.\n"
                ),
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )

        request = AgentRunRequest(
            task="INT-074 execution reviewer DOF rejection gate",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_EXECUTION_REVIEWER,
            start_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=request.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = EpisodeResponse.model_validate(
            await wait_for_episode_state(
                client,
                str(run.episode_id),
                timeout_s=180.0,
                predicate=lambda episode: _has_review_artifacts(
                    episode,
                    required_checklist_pairs=(("dof_deviation_justified", "fail"),),
                ),
            )
        )
        assert _has_review_artifacts(
            episode,
            required_checklist_pairs=(("dof_deviation_justified", "fail"),),
        ), episode

        review_traces = [
            trace for trace in (episode.traces or []) if trace.name == "review_decision"
        ]
        assert review_traces, "Expected review_decision trace for execution reviewer"
        latest_review_trace = max(review_traces, key=lambda trace: trace.id)

        inspect_media_traces = [
            trace for trace in (episode.traces or []) if _is_inspect_media_trace(trace)
        ]
        assert inspect_media_traces, (
            "Execution reviewer rejection must call inspect_media()."
        )
        dof_event_traces = [
            trace
            for trace in (episode.traces or [])
            if trace.name == "excessive_dof_detected"
        ]
        assert dof_event_traces, "Expected excessive_dof_detected trace"
        dof_event = next(
            trace
            for trace in dof_event_traces
            if _parse_event_payload(trace).get("reviewer_stage")
            == "engineering_execution_reviewer"
        )
        dof_payload = _parse_event_payload(dof_event)
        assert dof_payload["expected_minimal_engineering_dofs"] == [
            "tx",
            "ty",
            "tz",
        ], dof_payload
        assert (
            dof_payload["expected_minimal_engineering_dofs"]
            == dof_payload["expected_minimal_dofs"]
        ), dof_payload
        assert any(trace.id < dof_event.id for trace in inspect_media_traces), (
            "inspect_media must occur before the excessive_dof_detected event."
        )
        assert any(
            trace.id < latest_review_trace.id for trace in inspect_media_traces
        ), "inspect_media must occur before the final execution review decision."

        comments_path = "reviews/engineering-execution-review-comments-round-1.yaml"
        comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{episode.id}/assets/{comments_path}"
        )
        assert comments_resp.status_code == 200, comments_resp.text
        comments = yaml.safe_load(comments_resp.text)
        assert comments["checklist"]["dof_deviation_justified"] == "fail", comments

        # Regression: a non-DOF rejection must preserve the canonical key
        # without rewriting it to fail.
        render_gate_session = f"INT-210-{uuid.uuid4().hex[:8]}"
        render_gate_script = Path(
            "tests/integration/mock_responses/"
            "INT-182/engineer_coder/entry_01/01__script.py"
        ).read_text(encoding="utf-8")
        await seed_benchmark_assembly_definition(
            client,
            render_gate_session,
            benchmark_max_unit_cost_usd=250.0,
            benchmark_max_weight_g=2500.0,
            planner_target_max_unit_cost_usd=250.0,
            planner_target_max_weight_g=2500.0,
        )
        await seed_execution_reviewer_handover(
            client,
            session_id=render_gate_session,
            int_id="INT-210",
            script_content=render_gate_script,
        )
        await seed_current_revision_render_preview(
            client, session_id=render_gate_session
        )

        render_gate_request = AgentRunRequest(
            task="INT-074 non-DOF rejection checklist preservation",
            session_id=render_gate_session,
            agent_name=AgentName.ENGINEER_EXECUTION_REVIEWER,
            start_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
        render_gate_run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=render_gate_request.model_dump(mode="json"),
        )
        assert render_gate_run_resp.status_code == 202, render_gate_run_resp.text
        render_gate_run = AgentRunResponse.model_validate(render_gate_run_resp.json())

        render_gate_episode = EpisodeResponse.model_validate(
            await wait_for_episode_state(
                client,
                str(render_gate_run.episode_id),
                timeout_s=180.0,
                predicate=lambda episode: _has_review_artifacts(
                    episode,
                    required_checklist_pairs=(
                        ("dof_deviation_justified", "not_applicable"),
                    ),
                ),
            )
        )
        assert _has_review_artifacts(
            render_gate_episode,
            required_checklist_pairs=(("dof_deviation_justified", "not_applicable"),),
        ), render_gate_episode

        render_gate_review_traces = [
            trace
            for trace in (render_gate_episode.traces or [])
            if trace.name == "review_decision"
        ]
        assert render_gate_review_traces, "Expected review_decision trace"
        render_gate_comments_path = (
            "reviews/engineering-execution-review-comments-round-1.yaml"
        )
        render_gate_comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{render_gate_episode.id}/assets/"
            f"{render_gate_comments_path}"
        )
        assert render_gate_comments_resp.status_code == 200, (
            render_gate_comments_resp.text
        )
        render_gate_comments = yaml.safe_load(render_gate_comments_resp.text)
        assert "black/empty" in render_gate_comments["summary"], render_gate_comments
        assert (
            render_gate_comments["checklist"]["dof_deviation_justified"]
            == "not_applicable"
        ), render_gate_comments


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

        latest_episode = EpisodeResponse.model_validate(
            await wait_for_benchmark_state(
                client,
                session_id,
                timeout_s=120.0,
                terminal_statuses=set(),
                predicate=_benchmark_plan_review_artifacts_ready,
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

        assert latest_episode is not None
        traces = latest_episode.traces or []
        inspect_media_traces = [
            trace for trace in traces if _is_inspect_media_trace(trace)
        ]
        assert len(inspect_media_traces) >= 2, (
            "Benchmark plan reviewer rejection must inspect the latest revision "
            "render bundle before making a decision."
        )
        render_image_paths = {
            path.lstrip("/")
            for path in artifact_paths
            if path.startswith("renders/")
            and path.lower().endswith((".png", ".jpg", ".jpeg"))
        }
        media_event_contents = [
            trace.content or "" for trace in traces if trace.name == "media_inspection"
        ]
        inspected_render_paths = {
            path
            for path in render_image_paths
            if any(path in content for content in media_event_contents)
        }
        assert inspected_render_paths, (
            "Benchmark plan reviewer must inspect current-revision render images, "
            f"not arbitrary media. inspected_render_paths={sorted(inspected_render_paths)} "
            f"render_image_paths={sorted(render_image_paths)}"
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
        assert manifest.worker_session_id == "INT-203"
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
