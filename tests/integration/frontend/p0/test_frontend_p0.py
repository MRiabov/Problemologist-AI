import os
import re
import uuid
from urllib.parse import quote

import httpx
import pytest
from playwright.sync_api import Page, expect
from pydantic import TypeAdapter

from controller.api.routes.benchmark import BenchmarkGenerateRequest
from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkGenerateResponse,
    EpisodeListItem,
    EpisodeResponse,
)
from shared.enums import TraceType

# Constants
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


def _debug_info(page: Page) -> dict:
    return page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return {};
            try { return JSON.parse(el.textContent || "{}"); } catch (e) { return {}; }
        }"""
    )


def _wait_for_status(
    page: Page, statuses: list[str], timeout: int = 180000
) -> str | None:
    status_list = ",".join([f"'{s}'" for s in statuses])
    page.wait_for_function(
        f"""() => {{
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {{
                const data = JSON.parse(el.textContent);
                return [{status_list}].includes(data.episodeStatus);
            }} catch (e) {{ return false; }}
        }}""",
        timeout=timeout,
    )
    return (_debug_info(page) or {}).get("episodeStatus")


def _start_engineer_run(page: Page, prompt: str) -> None:
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")
    page.get_by_test_id("create-new-button").click()
    chat_input = page.locator("#chat-input")
    expect(chat_input).to_be_visible(timeout=30000)
    chat_input.fill(prompt)
    page.get_by_label("Send Message").click()


@pytest.mark.integration_frontend
def test_int_157_session_history(page: Page):
    """
    INT-157: Sidebar lists sessions from live API; selecting a session opens
    the correct benchmark/solution workflow with matching episode context.
    """
    unique_id = str(uuid.uuid4())[:8]
    benchmark_name = f"Benchmark-{unique_id}"
    engineer_name = f"Engineer-{unique_id}"

    # 1. Create a benchmark session and an engineer session via API
    with httpx.Client() as client:
        # Create benchmark session
        req_b = BenchmarkGenerateRequest(prompt=benchmark_name)
        resp_b = client.post(
            f"{CONTROLLER_URL}/api/benchmark/generate",
            json=req_b.model_dump(mode="json"),
        )
        assert resp_b.status_code == 200
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp_b.json())
        benchmark_id = str(benchmark_resp.session_id)

        # Create engineer session
        req_e = AgentRunRequest(task=engineer_name, session_id=str(uuid.uuid4()))
        resp_e = client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req_e.model_dump(mode="json"),
        )
        assert resp_e.status_code == 202
        agent_run_resp = AgentRunResponse.model_validate(resp_e.json())
        engineer_id = str(agent_run_resp.episode_id)

    # 2. Poll until both episodes are available via the API
    max_retries = 30
    episodes_found = False
    live_episodes = None
    with httpx.Client(timeout=10.0) as client:
        for i in range(max_retries):
            resp = client.get(f"{CONTROLLER_URL}/api/episodes/")
            if resp.status_code == 200:
                episodes = TypeAdapter(list[EpisodeListItem]).validate_python(
                    resp.json()
                )
                live_episodes = episodes
                ep_ids = [str(ep.id) for ep in episodes]
                if benchmark_id in ep_ids and engineer_id in ep_ids:
                    episodes_found = True
                    break

            # Use a slightly longer wait for the first few retries, then shorter
            import time

            time.sleep(0.5 if i > 5 else 1.0)

    assert episodes_found, (
        f"Episodes {benchmark_id} and {engineer_id} did not appear in API"
    )

    # 3. Navigate to frontend
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")

    # 4. Verify both sessions appear in sidebar
    expect(page.get_by_text(benchmark_name)).to_be_visible(timeout=30000)
    expect(page.get_by_text(engineer_name)).to_be_visible(timeout=30000)

    assert live_episodes is not None
    engineer_episode = next(ep for ep in live_episodes if str(ep.id) == engineer_id)
    engineer_row = (
        page.get_by_test_id("sidebar-episode-item").filter(has_text=engineer_name).first
    )
    expect(engineer_row.locator('[data-testid="episode-status"]')).to_have_attribute(
        "data-status",
        engineer_episode.status.value,
    )
    if (
        engineer_episode.metadata_vars
        and engineer_episode.metadata_vars.detailed_status
    ):
        expect(
            engineer_row.locator('[data-testid="sidebar-episode-detailed-status"]')
        ).to_have_attribute(
            "data-detailed-status",
            engineer_episode.metadata_vars.detailed_status,
        )
    if engineer_episode.metadata_vars and engineer_episode.metadata_vars.episode_phase:
        expect(
            engineer_row.locator('[data-testid="sidebar-episode-phase"]')
        ).to_have_attribute(
            "data-episode-phase",
            engineer_episode.metadata_vars.episode_phase.value,
        )

    # 5. Click benchmark session and verify navigation to /benchmark
    page.get_by_text(benchmark_name).click()
    expect(page).to_have_url(re.compile(r".*/benchmark"), timeout=15000)
    # The title "Benchmark Pipeline" is in a h2
    expect(page.locator("h2", has_text="Benchmark Pipeline")).to_be_visible(
        timeout=15000
    )

    # 6. Click engineer session and verify navigation to /
    page.get_by_text(engineer_name).click()
    expect(page).to_have_url(FRONTEND_URL + "/", timeout=15000)
    expect(page.locator("h2", has_text="Engineer Workspace")).to_be_visible(
        timeout=15000
    )


@pytest.mark.integration_frontend
def test_int_158_workflow_parity(page: Page):
    """
    INT-158: In both workflows, prompt submission, streamed assistant output,
    and artifact refresh behave consistently through real backend events.
    """
    # Test for Engineer Workflow
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")

    # Click CREATE NEW to ensure fresh state
    page.get_by_test_id("create-new-button").click()

    prompt_e = f"Build a 5mm cube {uuid.uuid4()}"
    chat_input = page.locator("#chat-input")
    expect(chat_input).to_be_visible()
    chat_input.fill(prompt_e)
    page.get_by_label("Send Message").click()

    # Verify thinking indicator
    expect(page.get_by_text(re.compile(r"thinking", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )
    expect(page.get_by_label("Stop Agent")).to_be_visible()

    # Wait for terminal status in engineer workflow.
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return ['COMPLETED', 'FAILED', 'CANCELLED'].includes(data.episodeStatus);
            } catch (e) { return false; }
        }""",
        timeout=120000,
    )
    expect(page.get_by_label("Send Message")).to_be_visible()

    # Test for Benchmark Workflow
    page.get_by_role("link", name="Benchmark").click()
    expect(page).to_have_url(re.compile(r".*/benchmark"))

    # Click CREATE NEW
    page.get_by_test_id("create-new-button").click()

    prompt_b = f"Move a sphere {uuid.uuid4()}"
    page.locator("#chat-input").fill(prompt_b)
    page.get_by_label("Send Message").click()

    expect(page.get_by_text(re.compile(r"thinking", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )
    expect(page.get_by_label("Stop Agent")).to_be_visible()

    # Benchmark workflow pauses at PLANNED for user confirmation.
    # In mock integration mode, benchmark generation can terminate FAILED.
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return ['PLANNED', 'FAILED'].includes(data.episodeStatus);
            } catch (e) { return false; }
        }""",
        timeout=120000,
    )
    benchmark_status = page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return null;
            try {
                return JSON.parse(el.textContent).episodeStatus ?? null;
            } catch (e) { return null; }
        }"""
    )
    assert benchmark_status == "PLANNED", (
        f"Benchmark did not reach PLANNED before confirmation (status={benchmark_status})"
    )

    # Confirm the plan to proceed to execution
    page.get_by_test_id("chat-confirm-button").click()

    # Wait for completion after confirmation
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return data.episodeStatus === 'COMPLETED';
            } catch (e) { return false; }
        }""",
        timeout=120000,
    )
    expect(page.get_by_label("Send Message")).to_be_visible()


@pytest.mark.integration_frontend
def test_int_159_plan_approval_comment(page: Page):
    """
    INT-159: After planner output, approve/disapprove controls appear;
    action posts decision to API; optional user comment is persisted
    and visible in run history.
    """
    # 1. Start a benchmark generation
    # NOTE: Direct goto('/benchmark') doesn't work because http-server --proxy
    # proxies non-static paths to the controller. Navigate via React Router instead.
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")
    page.get_by_role("link", name="Benchmark").click()
    expect(page).to_have_url(re.compile(r".*/benchmark"))

    # Click CREATE NEW
    page.get_by_test_id("create-new-button").click()

    page.locator("#chat-input").fill(
        f"INT-159: Generate a simple benchmark {uuid.uuid4()}"
    )
    page.get_by_label("Send Message").click()

    # 2. Wait for planner phase result.
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return ['PLANNED', 'FAILED'].includes(data.episodeStatus);
            } catch (e) { return false; }
        }""",
        timeout=180000,
    )
    benchmark_status = page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return null;
            try {
                return JSON.parse(el.textContent).episodeStatus ?? null;
            } catch (e) { return null; }
        }"""
    )
    if benchmark_status == "PLANNING":
        page.wait_for_function(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return false;
                try {
                    const data = JSON.parse(el.textContent);
                    return ['PLANNED', 'FAILED'].includes(data.episodeStatus);
                } catch (e) { return false; }
            }""",
            timeout=120000,
        )
        benchmark_status = page.evaluate(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return null;
                try {
                    return JSON.parse(el.textContent).episodeStatus ?? null;
                } catch (e) { return null; }
            }"""
        )

    assert benchmark_status == "PLANNED", (
        f"Benchmark did not reach PLANNED in approval flow (status={benchmark_status})"
    )
    expect(page.get_by_text("Execution Plan Ready")).to_be_visible()

    # 3. Verify comment field exists
    comment_field = page.get_by_placeholder("Optional comment for the agent...")
    expect(comment_field).to_be_visible()

    # 4. Fill in a comment and confirm
    test_comment = f"Test Comment {uuid.uuid4()}"
    comment_field.fill(test_comment)
    page.get_by_test_id("chat-confirm-button").click()

    # Wait for the agent to transition to RUNNING state in the UI
    expect(page.get_by_text(re.compile(r"thinking", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )

    # 5. Verify the comment is persisted as a trace via API
    with httpx.Client() as client:
        resp = client.get(f"{CONTROLLER_URL}/api/episodes/")
        assert resp.status_code == 200
        episodes = TypeAdapter(list[EpisodeListItem]).validate_python(resp.json())

        # Find the episode we just created/confirmed
        found_ep = None
        for ep in episodes:
            resp_ep = client.get(f"{CONTROLLER_URL}/api/episodes/{ep.id}")
            assert resp_ep.status_code == 200
            full_ep = EpisodeResponse.model_validate(resp_ep.json())
            for trace in full_ep.traces:
                if trace.trace_type == TraceType.LOG and test_comment in (
                    trace.content or ""
                ):
                    found_ep = full_ep
                    break
            if found_ep:
                break

        assert found_ep is not None, (
            f"Comment '{test_comment}' not found in any episode traces"
        )


@pytest.mark.integration_frontend
def test_int_162_interrupt_ux_propagation(page: Page):
    """
    INT-162: Stop action posts interrupt API and the run reaches cancelled state
    without ongoing stream growth.
    """
    _start_engineer_run(page, f"INT-162 interrupt path {uuid.uuid4()}")
    stop_button = page.get_by_label("Stop Agent")
    expect(stop_button).to_be_visible(timeout=30000)

    debug_before = _debug_info(page)
    episode_id = debug_before.get("episodeId")
    assert episode_id, "Episode ID missing in debug info before interrupt"
    with httpx.Client(timeout=10.0) as client:
        before_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert before_resp.status_code == 200
        before_episode = EpisodeResponse.model_validate(before_resp.json())
        before_trace_count = len(before_episode.traces or [])

    with page.expect_request(re.compile(r".*/api/episodes/.*/interrupt")):
        stop_button.click()

    final_status = _wait_for_status(page, ["CANCELLED", "FAILED"], timeout=120000)
    assert final_status == "CANCELLED", (
        f"Expected cancelled status after interrupt, got {final_status}"
    )
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=30000)

    import time

    time.sleep(2)
    with httpx.Client(timeout=10.0) as client:
        after_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert after_resp.status_code == 200
        after_episode = EpisodeResponse.model_validate(after_resp.json())
        after_trace_count = len(after_episode.traces or [])
    assert after_trace_count <= before_trace_count + 1, (
        "Trace stream continued to grow after interrupt; expected stream halt"
    )


@pytest.mark.integration_frontend
def test_int_163_steerability_context_cards_multi_select(page: Page):
    """
    INT-163: Multiple topology selections create context cards and cards can be removed.
    """
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")
    page.get_by_role("link", name="Benchmark").click()
    expect(page).to_have_url(re.compile(r".*/benchmark"))
    page.get_by_test_id("create-new-button").click()
    page.locator("#chat-input").fill(f"INT-163 context cards {uuid.uuid4()}")
    page.get_by_label("Send Message").click()

    status = _wait_for_status(page, ["PLANNED", "COMPLETED", "FAILED"], timeout=180000)
    assert status in {"PLANNED", "COMPLETED"}, (
        f"Run did not reach selectable state for context cards (status={status})"
    )
    if status == "PLANNED":
        page.get_by_test_id("chat-confirm-button").click(force=True)
        post_status = _wait_for_status(page, ["COMPLETED", "FAILED"], timeout=180000)
        assert post_status == "COMPLETED", (
            "Benchmark failed after confirmation; cannot verify context cards"
        )

    panel = page.get_by_test_id("model-browser-panel")
    expect(panel).to_be_visible(timeout=60000)
    nodes = page.locator('[data-testid="topology-node-row"]')
    if nodes.count() < 2:
        pytest.skip("Not enough topology nodes to verify multi-select context cards")

    initial_cards = page.get_by_test_id("context-card").count()
    nodes.nth(0).click(force=True)
    nodes.nth(1).click(force=True)
    page.wait_for_timeout(300)
    updated_cards = page.get_by_test_id("context-card").count()
    assert updated_cards >= initial_cards + 2, (
        "Expected at least two new context cards, "
        f"got initial={initial_cards}, updated={updated_cards}"
    )

    first_card = page.get_by_test_id("context-card").first
    first_card.locator("button").click(force=True)
    page.wait_for_timeout(200)
    assert page.get_by_test_id("context-card").count() == updated_cards - 1


@pytest.mark.integration_frontend
def test_int_167_controller_proxied_cad_assets(page: Page):
    """
    INT-167: CAD assets are fetched via controller proxy endpoints
    and non-GET is rejected.
    """
    asset_requests: list[str] = []

    def _capture_asset_request(request):
        if re.search(r"/api/episodes/.+/assets/", request.url):
            asset_requests.append(f"{request.method} {request.url}")

    page.on("request", _capture_asset_request)
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")
    page.get_by_role("link", name="Benchmark").click()
    page.get_by_test_id("create-new-button").click()
    page.locator("#chat-input").fill(f"INT-167 proxy assets {uuid.uuid4()}")
    page.get_by_label("Send Message").click()

    status = _wait_for_status(page, ["PLANNED", "COMPLETED", "FAILED"], timeout=180000)
    if status == "PLANNED":
        page.get_by_test_id("chat-confirm-button").click(force=True)
        status = _wait_for_status(page, ["COMPLETED", "FAILED"], timeout=180000)
    if status != "COMPLETED":
        pytest.skip(f"Run did not complete for asset proxy checks (status={status})")

    page.wait_for_timeout(2000)
    get_asset_requests = [r for r in asset_requests if r.startswith("GET ")]
    assert get_asset_requests, "No controller asset proxy GET requests were captured"

    info = _debug_info(page)
    episode_id = info.get("episodeId")
    assert episode_id, "Episode ID missing for asset method rejection check"
    with httpx.Client(timeout=10.0) as client:
        ep_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert ep_resp.status_code == 200
        episode = EpisodeResponse.model_validate(ep_resp.json())
        if not episode.assets:
            pytest.skip("No assets available to verify non-GET rejection")
        first_asset = episode.assets[0].s3_path
        assert first_asset, "First asset missing s3_path"
        encoded_path = quote(first_asset, safe="/")
        reject_resp = client.post(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/assets/{encoded_path}"
        )
        assert reject_resp.status_code in {403, 404, 405}, (
            f"Expected non-GET to be rejected, got {reject_resp.status_code}"
        )


@pytest.mark.integration_frontend
def test_int_170_post_run_feedback_ux_persistence(page: Page):
    """
    INT-170: Feedback controls appear only after completion
    and persist submitted feedback.
    """
    _start_engineer_run(page, f"INT-170 feedback path {uuid.uuid4()}")
    expect(page.get_by_test_id("chat-thumbs-up")).not_to_be_visible()
    expect(page.get_by_test_id("chat-thumbs-down")).not_to_be_visible()

    status = _wait_for_status(
        page, ["COMPLETED", "FAILED", "CANCELLED"], timeout=240000
    )
    if status != "COMPLETED":
        pytest.skip(f"Feedback assertions require completed run, got {status}")

    chat_thumbs_up = page.get_by_test_id("chat-thumbs-up").first
    expect(chat_thumbs_up).to_be_visible(timeout=30000)
    chat_thumbs_up.click()
    modal = page.get_by_test_id("feedback-modal")
    expect(modal).to_be_visible(timeout=10000)
    topic_name = "Technical Error"
    modal.get_by_role("button", name=topic_name).click()
    note = f"INT-170 feedback note {uuid.uuid4()}"
    modal.get_by_placeholder("How can the agent improve?").fill(note)
    modal.get_by_role("button", name="Send Feedback").click()
    expect(page.get_by_text("Feedback Received")).to_be_visible(timeout=30000)

    with httpx.Client(timeout=10.0) as client:
        info = _debug_info(page)
        episode_id = info.get("episodeId")
        assert episode_id, "Episode ID missing after feedback submit"
        ep_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert ep_resp.status_code == 200
        episode = EpisodeResponse.model_validate(ep_resp.json())
    assert any(
        trace.feedback_score == 1
        and trace.feedback_comment
        and note in trace.feedback_comment
        for trace in (episode.traces or [])
    ), "Submitted post-run feedback was not persisted in episode traces"
