import os
import re
import uuid

import httpx
import pytest
from playwright.sync_api import Page, expect
from pydantic import TypeAdapter

from controller.api.routes.benchmark import BenchmarkGenerateRequest
from controller.api.schemas import (
    AgentRunResponse,
    BenchmarkGenerateResponse,
    EpisodeListItem,
    EpisodeResponse,
)
from controller.api.tasks import AgentRunRequest
from shared.enums import TraceType

# Constants
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


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
    with httpx.Client(timeout=10.0) as client:
        for i in range(max_retries):
            resp = client.get(f"{CONTROLLER_URL}/api/episodes/")
            if resp.status_code == 200:
                episodes = TypeAdapter(list[EpisodeListItem]).validate_python(
                    resp.json()
                )
                ep_ids = [str(ep.id) for ep in episodes]
                if benchmark_id in ep_ids and engineer_id in ep_ids:
                    episodes_found = True
                    break
            import time

            time.sleep(1)

    assert episodes_found, (
        f"Episodes {benchmark_id} and {engineer_id} did not appear in API"
    )

    # 3. Navigate to frontend
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")

    # 4. Verify both sessions appear in sidebar
    expect(page.get_by_text(benchmark_name)).to_be_visible(timeout=30000)
    expect(page.get_by_text(engineer_name)).to_be_visible(timeout=30000)

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

    # Wait for completion (Send Message button returns)
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=120000)

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
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=120000)


@pytest.mark.integration_frontend
def test_int_159_plan_approval_comment(page: Page):
    """
    INT-159: After planner output, approve/disapprove controls appear;
    action posts decision to API; optional user comment is persisted
    and visible in run history.
    """
    # 1. Start a benchmark generation
    page.goto(f"{FRONTEND_URL}/benchmark")
    page.wait_for_load_state("networkidle")

    # Click CREATE NEW
    page.get_by_test_id("create-new-button").click()

    page.locator("#chat-input").fill(
        f"INT-159: Generate a simple benchmark {uuid.uuid4()}"
    )
    page.get_by_label("Send Message").click()

    # 2. Wait for the "Execution Plan Ready" card
    expect(page.get_by_text("Execution Plan Ready")).to_be_visible(timeout=180000)

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
