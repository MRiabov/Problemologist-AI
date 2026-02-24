import pytest
from playwright.sync_api import Page, expect
import uuid
import httpx
import os
import re

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
        resp_b = client.post(
            f"{CONTROLLER_URL}/benchmark/generate", json={"prompt": benchmark_name}
        )
        assert resp_b.status_code == 200
        benchmark_id = resp_b.json()["session_id"]

        # Create engineer session
        resp_e = client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": engineer_name, "session_id": str(uuid.uuid4())},
        )
        assert resp_e.status_code == 202
        engineer_id = resp_e.json()["episode_id"]

    # 2. Poll until both episodes are available via the API
    max_retries = 30
    episodes_found = False
    with httpx.Client(timeout=10.0) as client:
        for i in range(max_retries):
            resp = client.get(f"{CONTROLLER_URL}/episodes/")
            if resp.status_code == 200:
                ep_ids = [ep["id"] for ep in resp.json()]
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

    page.locator("#chat-input").fill(f"Generate a simple benchmark {uuid.uuid4()}")
    page.get_by_label("Send Message").click()

    # 2. Wait for the "Execution Plan Ready" card
    expect(page.get_by_text("Execution Plan Ready")).to_be_visible(timeout=180000)

    # 3. Verify comment field exists
    comment_field = page.get_by_placeholder("Optional comment for the agent...")
    expect(comment_field).to_be_visible()

    # 4. Fill in a comment and confirm
    test_comment = f"Test Comment {uuid.uuid4()}"
    comment_field.fill(test_comment)
    page.get_by_role("button", name="Confirm & Start").click()

    # Wait for the agent to transition to RUNNING state in the UI
    expect(page.get_by_text(re.compile(r"thinking", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )

    # 5. Verify the comment is persisted as a trace via API
    with httpx.Client() as client:
        episodes = client.get(f"{CONTROLLER_URL}/episodes/").json()
        # Find the episode we just created/confirmed
        found_ep = None
        for ep in episodes:
            full_ep = client.get(f"{CONTROLLER_URL}/episodes/{ep['id']}").json()
            for trace in full_ep.get("traces", []):
                if trace.get("trace_type") == "log" and test_comment in (
                    trace.get("content") or ""
                ):
                    found_ep = full_ep
                    break
            if found_ep:
                break

        assert found_ep is not None, (
            f"Comment '{test_comment}' not found in any episode traces"
        )
