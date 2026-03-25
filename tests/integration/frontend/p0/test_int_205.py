import os
import time
import uuid

import httpx
import pytest
import yaml
from playwright.sync_api import Page, expect

from controller.api.schemas import (
    AgentRunRequest,
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeCreateResponse,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus, EpisodeType
from shared.simulation.schemas import SimulatorBackendType

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


def _wait_for_episode_terminal(
    client: httpx.Client,
    episode_id: str,
    *,
    timeout_seconds: int = 240,
) -> EpisodeResponse:
    deadline = time.monotonic() + timeout_seconds
    last_status = None

    while time.monotonic() < deadline:
        response = client.get(f"/episodes/{episode_id}")
        assert response.status_code == 200, response.text
        episode = EpisodeResponse.model_validate(response.json())
        last_status = episode.status
        if episode.status in {
            EpisodeStatus.COMPLETED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
        }:
            return episode
        time.sleep(1.0)

    pytest.fail(
        f"Episode {episode_id} did not reach a terminal state (last={last_status})"
    )


def _reject_episode(client: httpx.Client, episode_id: str) -> EpisodeResponse:
    review_frontmatter = {
        "decision": "rejected",
        "comments": ["Retry lineage test rejection"],
        "evidence": {
            "stability_summary": {
                "batchWidth": 1,
                "successCount": 0,
                "successRate": 0.0,
                "isConsistent": False,
                "sceneBuildCount": 1,
                "backendRunCount": 1,
                "batchedExecution": False,
                "sceneSummaries": [
                    {
                        "sceneIndex": 1,
                        "success": False,
                        "summary": "Scene 1: failed validation.",
                        "failReason": "Deterministic retry lineage rejection.",
                        "failureMode": "STABILITY_ISSUE",
                    }
                ],
            },
            "stability_summary_source": "validation_results.json",
            "files_checked": ["plan.md", "validation_results.json"],
        },
    }
    review_content = (
        "---\n"
        + yaml.safe_dump(review_frontmatter, sort_keys=False).strip()
        + "\n---\n"
        + "Rejecting the episode for deterministic retry coverage.\n"
    )
    response = client.post(
        f"/episodes/{episode_id}/review",
        json={"review_content": review_content},
    )
    assert response.status_code == 200, response.text
    status_response = client.get(f"/episodes/{episode_id}")
    assert status_response.status_code == 200, status_response.text
    episode = EpisodeResponse.model_validate(status_response.json())
    assert episode.status == EpisodeStatus.FAILED
    return episode


def _create_benchmark(client: httpx.Client, prompt: str) -> str:
    request = BenchmarkGenerateRequest(
        prompt=prompt,
        backend=SimulatorBackendType.GENESIS,
    )
    response = client.post("/benchmark/generate", json=request.model_dump())
    assert response.status_code in (200, 202), response.text
    benchmark_session_id = BenchmarkGenerateResponse.model_validate(
        response.json()
    ).session_id

    benchmark_confirmed = False
    for _ in range(150):
        status_response = client.get(f"/benchmark/{benchmark_session_id}")
        if status_response.status_code == 404:
            time.sleep(1.0)
            continue
        assert status_response.status_code == 200, status_response.text
        benchmark_episode = EpisodeResponse.model_validate(status_response.json())
        if (
            benchmark_episode.status == EpisodeStatus.PLANNED
            and not benchmark_confirmed
        ):
            client.post(
                f"/benchmark/{benchmark_session_id}/confirm",
                json=ConfirmRequest(comment="Proceed").model_dump(),
            )
            benchmark_confirmed = True
        elif benchmark_episode.status == EpisodeStatus.COMPLETED:
            return str(benchmark_session_id)
        elif benchmark_episode.status == EpisodeStatus.FAILED:
            pytest.fail(
                "Benchmark generation failed during retry UI setup "
                f"(session_id={benchmark_session_id})."
            )
        time.sleep(2)

    pytest.fail("Benchmark generation failed or timed out during retry UI setup.")


def _seed_failed_engineer_episode(
    client: httpx.Client,
    *,
    benchmark_session_id: str,
    task: str,
    session_id: str,
    prior_episode_id: str | None = None,
    is_reused: bool | None = None,
    validation_logs: list[str] | None = None,
) -> str:
    metadata: dict[str, object] = {
        "benchmark_id": benchmark_session_id,
        "episode_type": EpisodeType.ENGINEER,
    }
    if prior_episode_id is not None:
        metadata["prior_episode_id"] = prior_episode_id
    if is_reused is not None:
        metadata["is_reused"] = is_reused
    if validation_logs is not None:
        metadata["validation_logs"] = validation_logs

    request = AgentRunRequest(
        task=task,
        session_id=session_id,
        metadata_vars=metadata,
    )
    response = client.post("/test/episodes", json=request.model_dump())
    assert response.status_code == 201, response.text
    episode_id = str(EpisodeCreateResponse.model_validate(response.json()).episode_id)

    episode = _reject_episode(client, episode_id)
    assert episode.metadata_vars is not None
    assert episode.metadata_vars.benchmark_id == benchmark_session_id
    assert episode.metadata_vars.episode_type == EpisodeType.ENGINEER
    if prior_episode_id is not None:
        assert episode.metadata_vars.prior_episode_id == prior_episode_id
    if is_reused is not None:
        assert episode.metadata_vars.is_reused is is_reused

    return episode_id


def _prepare_failed_engineer_run(
    benchmark_prompt: str,
    engineer_task: str,
    validation_logs: list[str] | None = None,
) -> tuple[str, str]:
    with httpx.Client(base_url=CONTROLLER_URL, timeout=300.0) as client:
        benchmark_session_id = _create_benchmark(client, benchmark_prompt)
        first_session_id = f"retry-{uuid.uuid4().hex[:8]}"
        first_episode_id = _seed_failed_engineer_episode(
            client,
            benchmark_session_id=benchmark_session_id,
            task=engineer_task,
            session_id=first_session_id,
            validation_logs=validation_logs,
        )
        return benchmark_session_id, first_episode_id


@pytest.mark.integration_frontend
@pytest.mark.allow_backend_errors(
    regexes=[
        "FAILED_RETRY_SCENARIO",
    ]
)
def test_int_205_failed_engineer_retry_revises_same_benchmark(page: Page):
    """
    INT-205: Failed engineer retry must remain on the same benchmark package,
    expose revision lineage in the workspace, and hide the retry control for
    non-failed episodes.
    """

    marker = uuid.uuid4().hex[:8]
    benchmark_prompt = (
        f"Create a simple benchmark setup for retry lineage testing {marker}"
    )
    engineer_task = f"Create an engineer handoff for retry lineage testing {marker}"
    validation_log = f"Validation log INT-205 {uuid.uuid4()}"

    benchmark_session_id, first_episode_id = _prepare_failed_engineer_run(
        benchmark_prompt,
        engineer_task,
        validation_logs=[validation_log],
    )

    page.add_init_script("localStorage.clear()")
    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")

    engineer_row = (
        page.get_by_test_id("sidebar-episode-item").filter(has_text=engineer_task).first
    )
    expect(engineer_row).to_be_visible(timeout=60000)
    engineer_row.click()

    page.wait_for_function(
        f"""() => {{
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {{
                const data = JSON.parse(el.textContent);
                return data.episodeId === "{first_episode_id}" &&
                    data.benchmarkId === "{benchmark_session_id}" &&
                    data.priorEpisodeId === null &&
                    data.isReused === null;
            }} catch (e) {{ return false; }}
        }}""",
        timeout=60000,
    )

    expect(page.get_by_test_id("failure-summary-container")).to_be_visible(
        timeout=15000
    )
    expect(page.get_by_test_id("retry-failed-episode-button")).to_be_visible(
        timeout=15000
    )
    expect(page.get_by_test_id("failure-summary-validation-logs")).to_contain_text(
        validation_log
    )
    expect(page.get_by_test_id("terminal-summary-validation-logs")).to_contain_text(
        validation_log
    )
    event_rows = page.get_by_test_id("run-event-row")
    expect(event_rows.first).to_be_visible(timeout=15000)
    expect(event_rows.filter(has_text="review_decision").first).to_be_visible(
        timeout=15000
    )

    benchmark_row = (
        page.get_by_test_id("sidebar-episode-item")
        .filter(has_text=benchmark_prompt)
        .first
    )
    expect(benchmark_row).to_be_visible(timeout=60000)
    benchmark_row.click()
    expect(page.get_by_test_id("retry-failed-episode-button")).to_have_count(0)

    engineer_row.click()

    captured_retry_payload: dict[str, object] = {}

    def _capture_retry_request(route):
        captured_retry_payload.clear()
        captured_retry_payload.update(route.request.post_data_json)
        route.fulfill(
            status=202,
            json={
                "status": "ACCEPTED",
                "message": "Retry captured for deterministic UI coverage",
                "episode_id": str(uuid.uuid4()),
            },
        )

    page.route("**/agent/run", _capture_retry_request)
    try:
        page.get_by_test_id("retry-failed-episode-button").click()
    finally:
        page.unroute("**/agent/run", _capture_retry_request)

    assert captured_retry_payload, "Retry request was not captured"
    assert captured_retry_payload["task"] == engineer_task
    assert captured_retry_payload["session_id"] != first_episode_id
    retry_metadata = captured_retry_payload["metadata_vars"]
    assert retry_metadata["benchmark_id"] == benchmark_session_id
    assert retry_metadata["prior_episode_id"] == first_episode_id
    assert retry_metadata["is_reused"] is True

    retry_session_id = str(captured_retry_payload["session_id"])
    with httpx.Client(base_url=CONTROLLER_URL, timeout=300.0) as client:
        second_episode_id = _seed_failed_engineer_episode(
            client,
            benchmark_session_id=benchmark_session_id,
            task=engineer_task,
            session_id=retry_session_id,
            prior_episode_id=first_episode_id,
            is_reused=True,
            validation_logs=[validation_log],
        )
        assert second_episode_id != first_episode_id

    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")

    engineer_row = (
        page.get_by_test_id("sidebar-episode-item").filter(has_text=engineer_task).first
    )
    expect(engineer_row).to_be_visible(timeout=60000)
    engineer_row.click()

    page.wait_for_function(
        f"""() => {{
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {{
                const data = JSON.parse(el.textContent);
                return data.benchmarkId === "{benchmark_session_id}" &&
                    data.priorEpisodeId === "{first_episode_id}" &&
                    data.isReused === true &&
                    data.revisionCount === 2;
            }} catch (e) {{ return false; }}
        }}""",
        timeout=60000,
    )

    expect(page.get_by_test_id("failure-summary-container")).to_be_visible(
        timeout=15000
    )
    expect(page.get_by_test_id("revision-summary-panel")).to_be_visible(timeout=15000)
    expect(page.get_by_test_id("revision-summary-item")).to_have_count(2)

    retry_row = (
        page.get_by_test_id("revision-summary-item").filter(has_text="Revision 2").first
    )
    expect(retry_row).to_be_visible(timeout=15000)
    retry_row.click()

    page.wait_for_function(
        f"""() => {{
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {{
                const data = JSON.parse(el.textContent);
                return data.episodeId !== "{first_episode_id}" &&
                    data.benchmarkId === "{benchmark_session_id}" &&
                    data.priorEpisodeId === "{first_episode_id}" &&
                    data.isReused === true;
            }} catch (e) {{ return false; }}
        }}""",
        timeout=60000,
    )

    expect(page.get_by_test_id("failure-summary-container")).to_be_visible(
        timeout=15000
    )
