import hashlib
import json
import time
import uuid
from pathlib import Path
from urllib.parse import urlparse

import httpx
import pytest
from playwright.sync_api import Page, expect

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus, EpisodeType, FailureReason
from shared.models.simulation import (
    MultiRunResult,
    SimulationFailure,
    SimulationMetrics,
    SimulationResult,
)
from shared.workers.schema import ValidationResultRecord, WriteFileRequest

CONTROLLER_URL = "http://localhost:18000"
FRONTEND_URL = "http://localhost:15173"
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


def _seed_peer_review_episode() -> tuple[
    str, str, EpisodeResponse, dict[str, dict[str, object]]
]:
    task = f"INT-189 solution evidence {uuid.uuid4()}"
    session_id = f"INT-189-{uuid.uuid4().hex[:8]}"
    benchmark_prompt = "Create a benchmark about stacking blocks."

    metadata_vars = {
        "is_reused": True,
        "episode_type": EpisodeType.ENGINEER,
    }

    with httpx.Client(base_url=CONTROLLER_URL, timeout=120.0) as client:
        benchmark_request = BenchmarkGenerateRequest(prompt=benchmark_prompt)
        benchmark_resp = client.post(
            "/api/benchmark/generate",
            json=benchmark_request.model_dump(mode="json"),
        )
        assert benchmark_resp.status_code in (200, 202), benchmark_resp.text
        benchmark_create = BenchmarkGenerateResponse.model_validate(
            benchmark_resp.json()
        )
        benchmark_id = str(benchmark_create.session_id)
        benchmark_episode_id = str(benchmark_create.episode_id)

        deadline = time.monotonic() + 180.0
        benchmark_episode = None
        confirmed = False
        while time.monotonic() < deadline:
            benchmark_state_resp = client.get(f"/api/benchmark/{benchmark_id}")
            if benchmark_state_resp.status_code == 404:
                time.sleep(1.0)
                continue
            assert benchmark_state_resp.status_code == 200, benchmark_state_resp.text
            benchmark_episode = EpisodeResponse.model_validate(
                benchmark_state_resp.json()
            )
            if benchmark_episode.status == EpisodeStatus.PLANNED and not confirmed:
                confirm_resp = client.post(
                    f"/api/benchmark/{benchmark_id}/confirm",
                    json=ConfirmRequest(comment="Proceed").model_dump(mode="json"),
                )
                assert confirm_resp.status_code in (200, 202), confirm_resp.text
                confirmed = True
            elif benchmark_episode.status == EpisodeStatus.COMPLETED:
                break
            elif benchmark_episode.status == EpisodeStatus.FAILED:
                raise AssertionError(
                    "Benchmark generation failed during setup "
                    f"(session_id={benchmark_id})."
                )
            time.sleep(1.0)

        assert benchmark_episode is not None, "Benchmark polling did not yield data."
        assert benchmark_episode.status == EpisodeStatus.COMPLETED

        glb_bytes = Path("assets/box.glb").read_bytes()
        cad_preview_bytes = Path(
            "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
        ).read_bytes()
        simulation_preview_bytes = Path(
            "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
        ).read_bytes()

        failure_metrics = SimulationMetrics(
            success=False,
            fail_reason="Scene 4 lost balance under jitter",
            fail_mode=FailureReason.STABILITY_ISSUE,
            failure=SimulationFailure(
                reason=FailureReason.STABILITY_ISSUE,
                detail="Scene 4 drifted off the support plane.",
            ),
        )
        verification_result = MultiRunResult(
            num_scenes=4,
            success_count=3,
            success_rate=0.75,
            is_consistent=False,
            individual_results=[
                SimulationMetrics(success=True),
                SimulationMetrics(success=True),
                SimulationMetrics(success=True),
                failure_metrics,
            ],
            fail_reasons=["Scene 4 lost balance under jitter"],
            scene_build_count=1,
            backend_run_count=1,
            batched_execution=True,
        )
        validation_record = ValidationResultRecord(
            success=True,
            message="Validation completed with batched jitter verification.",
            timestamp=time.time(),
            script_path="script.py",
            script_sha256=hashlib.sha256(b"print('peer review')\n").hexdigest(),
            verification_result=verification_result,
        )
        simulation_result = SimulationResult(
            success=True,
            summary="CAD and simulation previews are available for peer review.",
            render_paths=[
                "renders/cad_preview.png",
                "renders/simulation_preview.png",
            ],
            confidence="high",
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
            path="model.glb",
            content=glb_bytes,
            content_type="model/gltf-binary",
            bypass_agent_permissions=True,
        )
        _upload_workspace_file(
            client,
            session_id=session_id,
            path="renders/cad_preview.png",
            content=cad_preview_bytes,
            content_type="image/png",
            bypass_agent_permissions=True,
        )
        _upload_workspace_file(
            client,
            session_id=session_id,
            path="renders/simulation_preview.png",
            content=simulation_preview_bytes,
            content_type="image/png",
            bypass_agent_permissions=True,
        )

        metadata_vars["benchmark_id"] = benchmark_id
        metadata_vars["prior_episode_id"] = benchmark_episode_id

        request = AgentRunRequest(
            task=task,
            session_id=session_id,
            metadata_vars=metadata_vars,
        )

        run_resp = client.post(
            "/api/agent/run",
            json=request.model_dump(mode="json"),
        )
        assert run_resp.status_code in (200, 202), run_resp.text
        episode_id = str(AgentRunResponse.model_validate(run_resp.json()).episode_id)

        deadline = time.monotonic() + 240.0
        episode = None
        while time.monotonic() < deadline:
            episode_resp = client.get(f"/api/episodes/{episode_id}")
            assert episode_resp.status_code == 200, episode_resp.text
            episode = EpisodeResponse.model_validate(episode_resp.json())
            if episode.status in {
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
                EpisodeStatus.CANCELLED,
            }:
                break
            time.sleep(1.0)

        assert episode is not None, "Episode polling did not yield a response."
        assert episode.metadata_vars is not None
        assert episode.metadata_vars.benchmark_id == benchmark_id
        assert episode.metadata_vars.prior_episode_id == benchmark_episode_id
        assert episode.metadata_vars.is_reused is True
        assert episode.metadata_vars.episode_type == EpisodeType.ENGINEER
        artifact_paths = [asset.s3_path for asset in (episode.assets or [])]
        assert any(
            path.endswith("validation_results.json") for path in artifact_paths
        ), f"validation_results.json missing. Artifacts: {artifact_paths}"
        assert any(
            path.endswith("simulation_result.json") for path in artifact_paths
        ), f"simulation_result.json missing. Artifacts: {artifact_paths}"

    asset_payloads = {
        "validation_results.json": {
            "body": validation_record.model_dump_json(indent=2),
            "content_type": "application/json",
        },
        "simulation_result.json": {
            "body": simulation_result.model_dump_json(indent=2),
            "content_type": "application/json",
        },
        "model.glb": {
            "body": glb_bytes,
            "content_type": "model/gltf-binary",
        },
        "cad_preview.png": {
            "body": cad_preview_bytes,
            "content_type": "image/png",
        },
        "simulation_preview.png": {
            "body": simulation_preview_bytes,
            "content_type": "image/png",
        },
    }

    assert episode is not None
    return task, session_id, episode, asset_payloads


@pytest.mark.integration_frontend
def test_int_180_peer_review_card_surfaces_stability_and_review_actions(page: Page):
    task, session_id, episode, asset_payloads = _seed_peer_review_episode()

    captured_reviews: list[dict[str, object]] = []
    captured_assets: list[str] = []

    def handle_review_route(route):
        payload = route.request.post_data_json
        captured_reviews.append(payload)
        route.fulfill(
            status=200,
            json={"status": "SUCCESS", "decision": "APPROVED"},
        )

    def handle_asset_route(route):
        asset_name = Path(urlparse(route.request.url).path).name
        payload = asset_payloads.get(asset_name)
        if not payload:
            route.continue_()
            return
        captured_assets.append(asset_name)
        body = payload["body"]
        content_type = str(payload["content_type"])
        if isinstance(body, bytes):
            route.fulfill(status=200, body=body, headers={"Content-Type": content_type})
        else:
            route.fulfill(
                status=200, body=str(body), headers={"Content-Type": content_type}
            )

    page.route("**/api/episodes/*/assets/**", handle_asset_route)
    page.route("**/api/episodes/*/review", handle_review_route)
    page.add_init_script("localStorage.clear()")
    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")

    episode_row = (
        page.get_by_test_id("sidebar-episode-item").filter(has_text=task).first
    )
    expect(episode_row).to_be_visible(timeout=30000)
    episode_row.click()

    expect(page.get_by_test_id("peer-review-card")).to_be_visible(timeout=30000)
    expect(page.get_by_test_id("peer-review-status")).to_contain_text(
        "stability summary ready",
        timeout=30000,
    )
    expect(page.get_by_test_id("peer-review-stability-summary")).to_be_visible(
        timeout=30000
    )
    expect(page.get_by_test_id("peer-review-scene-1")).to_be_visible(timeout=30000)
    expect(page.get_by_test_id("peer-review-scene-4")).to_be_visible(timeout=30000)
    expect(page.get_by_test_id("sidebar-lineage-badges")).to_be_visible(timeout=30000)

    simulation_asset = next(
        asset
        for asset in episode.assets or []
        if asset.s3_path.endswith("simulation_result.json")
    )
    page.get_by_test_id(f"artifact-entry-{simulation_asset.id}").click()
    expect(page.get_by_test_id("artifact-active-file")).to_have_attribute(
        "data-artifact-name",
        "simulation_result.json",
        timeout=30000,
    )
    expect(page.get_by_test_id("verification-summary-panel")).to_be_visible(
        timeout=30000
    )
    expect(page.get_by_test_id("simulation-verification-card")).to_be_visible(
        timeout=30000
    )

    cad_asset = next(
        asset
        for asset in episode.assets or []
        if asset.s3_path.endswith("cad_preview.png")
    )
    page.get_by_test_id(f"artifact-entry-{cad_asset.id}").click()
    expect(page.get_by_test_id("artifact-media-view")).to_be_visible(timeout=30000)
    expect(page.get_by_test_id("verification-summary-panel")).to_be_visible(
        timeout=30000
    )

    simulation_preview_asset = next(
        asset
        for asset in episode.assets or []
        if asset.s3_path.endswith("simulation_preview.png")
    )
    page.get_by_test_id(f"artifact-entry-{simulation_preview_asset.id}").click()
    expect(page.get_by_test_id("artifact-media-view")).to_be_visible(timeout=30000)

    review_reason = "The batch is stable enough for release, but the reviewer still wants a revision."
    page.get_by_test_id("peer-review-reason").fill(review_reason)
    page.get_by_test_id("peer-review-reject-button").click()

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and len(captured_reviews) < 1:
        time.sleep(0.1)
    assert len(captured_reviews) >= 1, "Reject review request was not captured."
    reject_payload = captured_reviews[0]
    reject_content = str(reject_payload["review_content"])
    assert "decision: rejected" in reject_content
    assert review_reason in reject_content
    assert "validation_results.json" in reject_content
    assert "Stability summary:" in reject_content

    approve_reason = (
        "The batched verification is acceptable and the CAD preview is visible."
    )
    page.get_by_test_id("peer-review-reason").fill(approve_reason)
    page.get_by_test_id("peer-review-approve-button").click()

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and len(captured_reviews) < 2:
        time.sleep(0.1)
    assert len(captured_reviews) >= 2, "Approve review request was not captured."
    approve_payload = captured_reviews[1]
    approve_content = str(approve_payload["review_content"])
    assert "decision: approved" in approve_content
    assert approve_reason in approve_content
    assert "stability_summary:" in approve_content

    page.unroute("**/api/episodes/*/review", handle_review_route)
