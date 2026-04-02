import hashlib
import json
import re
import time
import uuid
from pathlib import Path

import httpx
import pytest
import yaml
from playwright.sync_api import Page, expect

from controller.api.schemas import AgentRunRequest, EpisodeResponse
from shared.enums import EpisodeStatus, TraceType
from shared.git_utils import repo_revision
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    CostTotals,
)
from shared.models.simulation import SimulationResult
from shared.workers.schema import (
    ReviewManifest,
    ValidationResultRecord,
    WriteFileRequest,
)

FRONTEND_URL = "http://localhost:15173"
CONTROLLER_URL = "http://localhost:18000"
WORKER_LIGHT_URL = "http://localhost:18001"


def _enum_value(value):
    return value.value if hasattr(value, "value") else value


def _asset_path(asset_path: str | Path) -> Path:
    return Path(str(asset_path).lstrip("/"))


def _write_workspace_file(
    client: httpx.Client,
    *,
    session_id: str,
    path: str,
    content: str,
    bypass_agent_permissions: bool = False,
) -> None:
    resp = client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path=path,
            content=content,
            overwrite=True,
            bypass_agent_permissions=bypass_agent_permissions,
        ).model_dump(),
        headers={
            "X-Session-ID": session_id,
            **({"X-System-FS-Bypass": "1"} if bypass_agent_permissions else {}),
        },
        timeout=30.0,
    )
    assert resp.status_code == 200, resp.text


def _upload_workspace_file(
    client: httpx.Client,
    *,
    session_id: str,
    path: str,
    content: bytes,
    content_type: str,
    bypass_agent_permissions: bool = False,
) -> None:
    resp = client.post(
        f"{WORKER_LIGHT_URL}/fs/upload_file",
        data={
            "path": path,
            "bypass_agent_permissions": json.dumps(bypass_agent_permissions),
        },
        files={
            "file": (
                Path(path).name,
                content,
                content_type,
            )
        },
        headers={
            "X-Session-ID": session_id,
            **({"X-System-FS-Bypass": "1"} if bypass_agent_permissions else {}),
        },
        timeout=30.0,
    )
    assert resp.status_code == 200, resp.text


def _seed_engineer_handoff(client: httpx.Client, *, session_id: str) -> None:
    script_path = Path(
        "tests/integration/mock_responses/INT-181/engineer_coder/entry_01/01__script.py"
    )
    script_content = script_path.read_text(encoding="utf-8")
    script_sha256 = hashlib.sha256(script_content.encode("utf-8")).hexdigest()
    seed_ts = time.time()
    revision = repo_revision(Path(__file__).resolve().parents[4])
    render_path = "renders/render_preview.png"
    stale_model_path = "zzz_stale_geometry.glb"
    current_model_path = "aaa_current_geometry.glb"
    current_video_path = "current_simulation.mp4"
    render_content = Path(
        "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
    ).read_bytes()
    stale_model_content = Path("assets/part_0.glb").read_bytes()
    current_model_content = Path("assets/target_box.glb").read_bytes()
    current_video_content = b"not-a-real-mp4-but-sufficient-for-asset-routing"

    assembly = AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=200.0,
            benchmark_max_weight_g=1000.0,
            planner_target_max_unit_cost_usd=200.0,
            planner_target_max_weight_g=1000.0,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[],
        totals=CostTotals(
            estimated_unit_cost_usd=0.0,
            estimated_weight_g=0.0,
            estimate_confidence="medium",
        ),
    )
    validation_record = ValidationResultRecord(
        success=True,
        message="Validation completed",
        timestamp=seed_ts,
        script_path="script.py",
        script_sha256=script_sha256,
    )
    simulation_result = SimulationResult(
        success=True,
        summary="Goal achieved in green zone.",
        render_paths=[render_path],
        confidence="high",
    )
    review_manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage="engineering_execution_reviewer",
        session_id=session_id,
        script_path="script.py",
        script_sha256=script_sha256,
        validation_success=True,
        validation_timestamp=seed_ts,
        simulation_success=True,
        simulation_summary="Goal achieved in green zone.",
        simulation_timestamp=seed_ts,
        goal_reached=True,
        revision=revision,
        renders=[render_path],
        mjcf_path="renders/scene.xml",
        cad_path="renders/model.step",
        objectives_path="renders/benchmark_definition.yaml",
        assembly_definition_path="renders/assembly_definition.yaml",
    )

    _write_workspace_file(
        client,
        session_id=session_id,
        path="benchmark_assembly_definition.yaml",
        content=yaml.safe_dump(
            assembly.model_dump(mode="json", by_alias=True),
            sort_keys=False,
        ),
    )
    _write_workspace_file(
        client,
        session_id=session_id,
        path="validation_results.json",
        content=validation_record.model_dump_json(indent=2),
        bypass_agent_permissions=True,
    )
    _write_workspace_file(
        client,
        session_id=session_id,
        path="simulation_result.json",
        content=simulation_result.model_dump_json(indent=2),
        bypass_agent_permissions=True,
    )
    _upload_workspace_file(
        client,
        session_id=session_id,
        path=stale_model_path,
        content=stale_model_content,
        content_type="model/gltf-binary",
        bypass_agent_permissions=True,
    )
    _upload_workspace_file(
        client,
        session_id=session_id,
        path=current_model_path,
        content=current_model_content,
        content_type="model/gltf-binary",
        bypass_agent_permissions=True,
    )
    _upload_workspace_file(
        client,
        session_id=session_id,
        path=current_video_path,
        content=current_video_content,
        content_type="video/mp4",
        bypass_agent_permissions=True,
    )
    _upload_workspace_file(
        client,
        session_id=session_id,
        path=render_path,
        content=render_content,
        content_type="image/png",
        bypass_agent_permissions=True,
    )
    _write_workspace_file(
        client,
        session_id=session_id,
        path=".manifests/engineering_execution_review_manifest.json",
        content=review_manifest.model_dump_json(indent=2),
        bypass_agent_permissions=True,
    )


def _start_and_wait_for_engineer_episode(task: str) -> EpisodeResponse:
    session_id = f"INT-189-{uuid.uuid4().hex[:8]}"
    with httpx.Client(timeout=120.0) as client:
        _seed_engineer_handoff(client, session_id=session_id)

        run_request = AgentRunRequest(
            task=task,
            session_id=session_id,
        )
        run_resp = client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=run_request.model_dump(mode="json"),
        )
        assert run_resp.status_code in (200, 202), run_resp.text
        episode_id = run_resp.json()["episode_id"]

        deadline = time.monotonic() + 240.0
        while time.monotonic() < deadline:
            resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
            assert resp.status_code == 200, resp.text
            payload = EpisodeResponse.model_validate(resp.json())
            if payload.status in {
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
                EpisodeStatus.CANCELLED,
            }:
                return payload
            time.sleep(1.0)

        raise AssertionError(f"Episode {episode_id} did not reach terminal status")


@pytest.mark.integration_frontend
@pytest.mark.allow_backend_errors(
    regexes=[
        "filesystem_permission_denied.*benchmark_definition.yaml",
        "benchmark_definition_yaml_invalid",
        "engineer_planner_dspy_failed",
    ]
)
def test_int_189_engineer_run_defaults_to_solution_evidence(page: Page):
    """
    INT-189: A finished engineer run should open terminal metadata and solution
    evidence by default instead of the planner draft.
    """
    task = f"INT-189 solution evidence {uuid.uuid4()}"
    episode = _start_and_wait_for_engineer_episode(task)
    metadata = episode.metadata_vars
    assert metadata is not None
    assert metadata.episode_type == "engineer"
    assert metadata.detailed_status in {
        EpisodeStatus.COMPLETED.value,
        EpisodeStatus.FAILED.value,
    }
    assert metadata.terminal_reason is not None
    if episode.status == EpisodeStatus.FAILED:
        assert metadata.failure_class is not None
    else:
        assert metadata.failure_class is None

    artifact_paths = [_asset_path(asset.s3_path) for asset in (episode.assets or [])]
    model_assets = [
        asset
        for asset in (episode.assets or [])
        if _asset_path(asset.s3_path).suffix == ".glb"
    ]
    assert Path("simulation_result.json") in artifact_paths, (
        f"simulation_result.json missing. Artifacts: {artifact_paths}"
    )
    assert Path("validation_results.json") in artifact_paths, (
        f"validation_results.json missing. Artifacts: {artifact_paths}"
    )
    assert Path("current_simulation.mp4") in artifact_paths, (
        f"current_simulation.mp4 missing. Artifacts: {artifact_paths}"
    )

    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")
    episode_item = (
        page.get_by_test_id("sidebar-episode-item").filter(has_text=task).first
    )
    expect(episode_item).to_be_visible(timeout=30000)
    expect(episode_item.locator('[data-testid="episode-status"]')).to_have_attribute(
        "data-status",
        _enum_value(episode.status),
    )
    expect(
        episode_item.locator('[data-testid="sidebar-episode-detailed-status"]')
    ).to_have_attribute("data-detailed-status", metadata.detailed_status)
    expect(
        episode_item.locator('[data-testid="sidebar-episode-terminal-reason"]')
    ).to_have_attribute(
        "data-terminal-reason",
        _enum_value(metadata.terminal_reason),
    )
    if metadata.failure_class is not None:
        expect(
            episode_item.locator('[data-testid="sidebar-episode-failure-class"]')
        ).to_have_attribute(
            "data-failure-class",
            _enum_value(metadata.failure_class),
        )
    if metadata.episode_phase is not None:
        expect(
            episode_item.locator('[data-testid="sidebar-episode-phase"]')
        ).to_have_attribute(
            "data-episode-phase",
            _enum_value(metadata.episode_phase),
        )
    expect(
        episode_item.locator('[data-testid="sidebar-episode-progress"]')
    ).to_have_attribute("data-progress", "100")
    episode_item.click()

    context_usage = (metadata.additional_info or {}).get("context_usage")
    if isinstance(context_usage, dict):
        used_tokens = context_usage.get("used_tokens")
        max_tokens = context_usage.get("max_tokens")
        if used_tokens is not None and max_tokens is not None:
            used_tokens = int(used_tokens)
            max_tokens = int(max_tokens)
            expected_pct = (used_tokens / max_tokens) * 100 if max_tokens else 0.0
            expect(page.get_by_test_id("context-usage-indicator")).to_be_visible(
                timeout=30000
            )
            expect(page.get_by_test_id("context-usage-indicator")).to_contain_text(
                f"Ctx {used_tokens:,} / {max_tokens:,} ({expected_pct:.1f}%)"
            )

    expect(page.get_by_test_id("terminal-summary-block")).to_be_visible(timeout=30000)
    expect(page.get_by_test_id("terminal-summary-detailed-status")).to_be_visible()
    expect(page.get_by_test_id("terminal-summary-terminal-reason")).to_be_visible()
    expect(page.get_by_test_id("terminal-summary-failure-class")).to_be_visible()

    viewport_debug = page.get_by_test_id("unified-debug-info").text_content()
    assert viewport_debug is not None
    viewport_state = json.loads(viewport_debug)
    terminal_reason = page.get_by_test_id(
        "terminal-summary-terminal-reason"
    ).inner_text()
    assert terminal_reason, "Expected terminal reason to be visible in workspace."
    assert viewport_state["modelUrlsCount"] == 1, viewport_state
    assert viewport_state["modelAssetPath"] == model_assets[0].s3_path, viewport_state
    assert Path(viewport_state["videoAssetPath"]) == Path("current_simulation.mp4"), (
        viewport_state
    )

    artifact_debug = page.get_by_test_id("artifact-debug-info").text_content()
    assert artifact_debug is not None
    artifact_state = json.loads(artifact_debug)
    assert (
        artifact_state["selectedSolutionEvidenceArtifact"]["name"]
        == "simulation_result.json"
    )
    assert artifact_state["activeArtifactName"] == "simulation_result.json"
    assert Path(artifact_state["activeArtifactPath"]) == Path("simulation_result.json")
    assert len(model_assets) >= 2, (
        f"Expected multiple GLB assets to prove latest-media binding, got: "
        f"{[asset.s3_path for asset in (episode.assets or [])]}"
    )
    assert (
        artifact_state["latestMediaBundle"]["model"]["path"] == model_assets[0].s3_path
    ), artifact_state["latestMediaBundle"]
    assert (
        artifact_state["latestMediaBundle"]["model"]["name"]
        == Path(model_assets[0].s3_path).name
    )
    assert Path(
        artifact_state["latestMediaBundle"]["solutionEvidence"]["path"]
    ) == Path("simulation_result.json")
    assert artifact_state["traceEpisodeId"] == viewport_state["mediaEpisodeId"], (
        artifact_state
    )
    assert artifact_state["traceCount"] > 0, artifact_state

    expect(page.get_by_test_id("design-viewer-video")).to_have_attribute(
        "aria-hidden",
        "false",
        timeout=30000,
    )
    expect(page.get_by_test_id("design-viewer-3d-panel")).to_have_attribute(
        "aria-hidden",
        "true",
        timeout=30000,
    )
    page.get_by_test_id("design-viewer-mode-3d").click()
    expect(page.get_by_test_id("design-viewer-3d-panel")).to_have_attribute(
        "aria-hidden",
        "false",
        timeout=30000,
    )
    page.get_by_title("Part Selection").click()
    expect(page.get_by_title("Part Selection")).to_have_class(re.compile(r"bg-primary"))
    page.get_by_test_id("design-viewer-mode-video").click()
    expect(page.get_by_test_id("design-viewer-video")).to_have_attribute(
        "aria-hidden",
        "false",
        timeout=30000,
    )
    page.get_by_test_id("design-viewer-mode-3d").click()
    expect(page.get_by_title("Part Selection")).to_have_class(re.compile(r"bg-primary"))

    active_artifact = page.get_by_test_id("artifact-active-file")
    expect(active_artifact).to_have_attribute(
        "data-artifact-name", "simulation_result.json"
    )
    expect(active_artifact).to_have_attribute(
        "data-artifact-path",
        artifact_state["activeArtifactPath"],
    )

    expect(page.get_by_test_id("simulation-results-root")).to_be_visible()

    image_asset = next(
        asset
        for asset in (episode.assets or [])
        if Path(asset.s3_path) == Path("renders/render_preview.png")
    )
    page.get_by_test_id(f"artifact-entry-{image_asset.id}").click()
    expect(page.get_by_test_id("artifact-media-view")).to_be_visible()
    expect(page.get_by_test_id("artifact-active-file")).to_have_attribute(
        "data-artifact-name",
        "render_preview.png",
    )
    expect(page.get_by_test_id("artifact-active-file")).to_have_attribute(
        "data-artifact-path",
        image_asset.s3_path,
    )


@pytest.mark.integration_frontend
def test_int_189_persisted_event_metadata_renders_when_content_is_empty_json(
    page: Page,
):
    """
    INT-189 regression: generic EVENT rows should prefer persisted metadata when
    the stored event body is just empty JSON.
    """
    episode_task = f"INT-189 empty event body {uuid.uuid4()}"
    session_id = f"INT-189-EVENT-{uuid.uuid4().hex[:8]}"

    with httpx.Client(timeout=120.0) as client:
        create_resp = client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json=AgentRunRequest(
                task=episode_task,
                session_id=session_id,
            ).model_dump(mode="json"),
        )
        assert create_resp.status_code == 201, create_resp.text
        episode_id = create_resp.json()["episode_id"]

        detail_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert detail_resp.status_code == 200, detail_resp.text
        episode = EpisodeResponse.model_validate(detail_resp.json())

    payload = episode.model_dump(mode="json")
    payload["status"] = EpisodeStatus.COMPLETED.value
    payload["traces"] = [
        {
            "id": 910000,
            "user_session_id": None,
            "langfuse_trace_id": None,
            "simulation_run_id": None,
            "cots_query_id": None,
            "review_id": None,
            "trace_type": TraceType.EVENT.value,
            "name": "circuit_simulation",
            "content": "{}",
            "metadata_vars": {
                "event_type": "circuit_simulation",
                "motor_states": {
                    "motor_alpha": "on",
                    "motor_beta": "off",
                },
                "additional_info": {
                    "motor_states": {
                        "motor_alpha": "on",
                        "motor_beta": "off",
                    }
                },
            },
            "feedback_score": None,
            "feedback_comment": None,
            "created_at": episode.created_at.isoformat(),
        }
    ]

    def _mock_episode_detail(route):
        route.fulfill(status=200, json=payload)

    page.route(f"**/api/episodes/{episode_id}", _mock_episode_detail)
    try:
        page.goto(FRONTEND_URL, timeout=60000)
        page.evaluate(
            "(selectedEpisodeId) => localStorage.setItem('selectedEpisodeId', selectedEpisodeId)",
            episode_id,
        )
        page.reload()
        page.wait_for_function(
            """(expectedEpisodeId) => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return false;
                try {
                    const data = JSON.parse(el.textContent);
                    return data.episodeId === expectedEpisodeId;
                } catch (e) { return false; }
            }""",
            arg=episode_id,
            timeout=60000,
        )

        event_row = (
            page.get_by_test_id("run-event-row")
            .filter(has_text="circuit_simulation")
            .first
        )
        expect(event_row).to_be_visible(timeout=30000)
        expect(event_row).to_contain_text("motor_alpha")
        expect(event_row).to_contain_text("motor_beta")
        expect(event_row).not_to_contain_text("{}")
    finally:
        page.unroute(f"**/api/episodes/{episode_id}", _mock_episode_detail)
