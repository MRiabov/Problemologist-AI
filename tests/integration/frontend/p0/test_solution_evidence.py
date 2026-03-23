import hashlib
import json
import time
import uuid
from pathlib import Path

import httpx
import pytest
import yaml
from playwright.sync_api import Page, expect

from controller.api.schemas import AgentRunRequest, EpisodeResponse
from shared.enums import EpisodeStatus
from shared.git_utils import repo_revision
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    CostTotals,
)
from shared.models.simulation import SimulationResult
from shared.workers.schema import ReviewManifest, ValidationResultRecord, WriteFileRequest

FRONTEND_URL = "http://localhost:15173"
CONTROLLER_URL = "http://localhost:18000"
WORKER_LIGHT_URL = "http://localhost:18001"


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
    render_content = Path(
        "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
    ).read_bytes()

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

    artifact_paths = [asset.s3_path for asset in (episode.assets or [])]
    assert any(path.endswith("simulation_result.json") for path in artifact_paths), (
        f"simulation_result.json missing. Artifacts: {artifact_paths}"
    )
    assert any(path.endswith("validation_results.json") for path in artifact_paths), (
        f"validation_results.json missing. Artifacts: {artifact_paths}"
    )

    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")
    episode_item = page.get_by_test_id("sidebar-episode-item").filter(has_text=task).first
    expect(episode_item).to_be_visible(timeout=30000)
    episode_item.click()

    expect(page.get_by_test_id("terminal-summary-block")).to_be_visible(
        timeout=30000
    )
    expect(page.get_by_test_id("terminal-summary-detailed-status")).to_be_visible()
    expect(page.get_by_test_id("terminal-summary-terminal-reason")).to_be_visible()
    expect(page.get_by_test_id("terminal-summary-failure-class")).to_be_visible()

    terminal_reason = page.get_by_test_id("terminal-summary-terminal-reason").inner_text()
    assert terminal_reason, "Expected terminal reason to be visible in workspace."

    artifact_debug = page.get_by_test_id("artifact-debug-info").text_content()
    assert artifact_debug is not None
    artifact_state = json.loads(artifact_debug)
    assert artifact_state["selectedSolutionEvidenceArtifact"]["name"] == "simulation_result.json"
    assert artifact_state["activeArtifactName"] == "simulation_result.json"
    assert artifact_state["activeArtifactPath"].endswith("simulation_result.json")

    active_artifact = page.get_by_test_id("artifact-active-file")
    expect(active_artifact).to_have_attribute("data-artifact-name", "simulation_result.json")
    expect(active_artifact).to_have_attribute(
        "data-artifact-path",
        artifact_state["activeArtifactPath"],
    )

    expect(page.get_by_test_id("simulation-results-root")).to_be_visible()

    image_asset = next(
        asset for asset in (episode.assets or []) if asset.s3_path.endswith("render_preview.png")
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
