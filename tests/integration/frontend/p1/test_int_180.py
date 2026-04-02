import asyncio
import hashlib
import json
import threading
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path
from urllib.parse import urlparse

import httpx
import pytest
from playwright.sync_api import Page, expect

from controller.api.schemas import EpisodeResponse
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode
from shared.enums import AssetType, EpisodeStatus, EpisodeType, FailureReason
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
    task = f"Review peer solution stability: {uuid.uuid4()}"
    worker_session_id = f"INT-180-{uuid.uuid4().hex[:8]}"
    episode_id = uuid.uuid4()

    with httpx.Client(base_url=CONTROLLER_URL, timeout=120.0) as client:
        glb_bytes = Path("assets/box.glb").read_bytes()
        cad_preview_bytes = Path(
            "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
        ).read_bytes()
        simulation_preview_bytes = Path(
            "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
        ).read_bytes()
        render_probe_bytes = cad_preview_bytes
        benchmark_id = f"BENCH-{uuid.uuid4().hex[:8]}"

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
            session_id=worker_session_id,
            path="validation_results.json",
            content=validation_record.model_dump_json(indent=2),
            bypass_agent_permissions=True,
        )
        _write_workspace_file(
            client,
            session_id=worker_session_id,
            path="simulation_result.json",
            content=simulation_result.model_dump_json(indent=2),
            bypass_agent_permissions=True,
        )
        _upload_workspace_file(
            client,
            session_id=worker_session_id,
            path="model.glb",
            content=glb_bytes,
            content_type="model/gltf-binary",
            bypass_agent_permissions=True,
        )
        _upload_workspace_file(
            client,
            session_id=worker_session_id,
            path="renders/render_e45_a45.png",
            content=render_probe_bytes,
            content_type="image/png",
            bypass_agent_permissions=True,
        )
        _upload_workspace_file(
            client,
            session_id=worker_session_id,
            path="renders/cad_preview.png",
            content=cad_preview_bytes,
            content_type="image/png",
            bypass_agent_permissions=True,
        )
        _upload_workspace_file(
            client,
            session_id=worker_session_id,
            path="renders/simulation_preview.png",
            content=simulation_preview_bytes,
            content_type="image/png",
            bypass_agent_permissions=True,
        )

        async def _insert_episode() -> None:
            session_factory = get_sessionmaker()
            async with session_factory() as db:
                db.add(
                    Episode(
                        id=episode_id,
                        task=task,
                        status=EpisodeStatus.COMPLETED,
                        metadata_vars={
                            "benchmark_id": benchmark_id,
                            "worker_session_id": worker_session_id,
                            "episode_type": EpisodeType.ENGINEER.value,
                            "is_reused": True,
                        },
                    )
                )
                db.add_all(
                    [
                        Asset(
                            episode_id=episode_id,
                            asset_type=AssetType.OTHER,
                            s3_path="validation_results.json",
                            content=None,
                        ),
                        Asset(
                            episode_id=episode_id,
                            asset_type=AssetType.OTHER,
                            s3_path="simulation_result.json",
                            content=None,
                        ),
                        Asset(
                            episode_id=episode_id,
                            asset_type=AssetType.GLB,
                            s3_path="model.glb",
                            content=None,
                        ),
                        Asset(
                            episode_id=episode_id,
                            asset_type=AssetType.IMAGE,
                            s3_path="renders/render_e45_a45.png",
                            content=None,
                        ),
                        Asset(
                            episode_id=episode_id,
                            asset_type=AssetType.IMAGE,
                            s3_path="renders/cad_preview.png",
                            content=None,
                        ),
                        Asset(
                            episode_id=episode_id,
                            asset_type=AssetType.IMAGE,
                            s3_path="renders/simulation_preview.png",
                            content=None,
                        ),
                    ]
                )
                await db.commit()

        insert_error: list[BaseException] = []

        def _run_insert_episode() -> None:
            try:
                asyncio.run(_insert_episode())
            except BaseException as exc:  # pragma: no cover - surfaced in test failure
                insert_error.append(exc)

        insert_thread = threading.Thread(target=_run_insert_episode, daemon=True)
        insert_thread.start()
        insert_thread.join()
        if insert_error:
            raise insert_error[0]

        now = datetime.now(UTC)
        episode = EpisodeResponse.model_validate(
            {
                "id": str(episode_id),
                "task": task,
                "status": EpisodeStatus.COMPLETED,
                "created_at": now,
                "updated_at": now,
                "metadata_vars": {
                    "benchmark_id": benchmark_id,
                    "worker_session_id": worker_session_id,
                    "episode_type": EpisodeType.ENGINEER.value,
                    "is_reused": True,
                },
                "todo_list": {"completed": True},
                "validation_logs": [],
                "last_trace_id": None,
                "traces": [],
                "assets": [
                    {
                        "id": 1,
                        "asset_type": AssetType.OTHER,
                        "s3_path": "validation_results.json",
                        "content": validation_record.model_dump_json(indent=2),
                        "created_at": now,
                    },
                    {
                        "id": 2,
                        "asset_type": AssetType.OTHER,
                        "s3_path": "simulation_result.json",
                        "content": simulation_result.model_dump_json(indent=2),
                        "created_at": now,
                    },
                    {
                        "id": 3,
                        "asset_type": AssetType.GLB,
                        "s3_path": "model.glb",
                        "content": None,
                        "created_at": now,
                    },
                    {
                        "id": 4,
                        "asset_type": AssetType.IMAGE,
                        "s3_path": "renders/render_e45_a45.png",
                        "content": None,
                        "created_at": now,
                    },
                    {
                        "id": 5,
                        "asset_type": AssetType.IMAGE,
                        "s3_path": "renders/cad_preview.png",
                        "content": None,
                        "created_at": now,
                    },
                    {
                        "id": 6,
                        "asset_type": AssetType.IMAGE,
                        "s3_path": "renders/simulation_preview.png",
                        "content": None,
                        "created_at": now,
                    },
                ],
            }
        )

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
        "render_e45_a45.png": {
            "body": render_probe_bytes,
            "content_type": "image/png",
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
    return task, worker_session_id, episode, asset_payloads


@pytest.mark.integration_frontend
def test_int_180_peer_review_card_surfaces_stability_and_review_actions(page: Page):
    task, session_id, episode, asset_payloads = _seed_peer_review_episode()

    captured_reviews: list[dict[str, object]] = []
    captured_assets: list[str] = []
    episode_payload = episode.model_dump(mode="json")
    episode_list_payload = [
        {
            key: value
            for key, value in episode_payload.items()
            if key not in {"traces", "assets"}
        }
    ]

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

    def handle_episode_list_route(route):
        route.fulfill(status=200, json=episode_list_payload)

    def handle_episode_detail_route(route):
        route.fulfill(status=200, json=episode_payload)

    page.route("**/api/episodes", handle_episode_list_route)
    page.route("**/api/episodes/", handle_episode_list_route)
    page.route(f"**/api/episodes/{episode.id}", handle_episode_detail_route)
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
        if Path(asset.s3_path) == Path("simulation_result.json")
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
        if Path(asset.s3_path) == Path("renders/cad_preview.png")
    )
    page.get_by_test_id(f"artifact-entry-{cad_asset.id}").click()
    expect(page.get_by_test_id("artifact-media-view")).to_be_visible(timeout=30000)
    expect(page.get_by_test_id("verification-summary-panel")).to_be_visible(
        timeout=30000
    )

    simulation_preview_asset = next(
        asset
        for asset in episode.assets or []
        if Path(asset.s3_path) == Path("renders/simulation_preview.png")
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
    assert "stability_summary:" in approve_content

    page.unroute("**/api/episodes/*/review", handle_review_route)
