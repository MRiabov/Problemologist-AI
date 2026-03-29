import os
import re
import uuid

import pytest
from playwright.sync_api import Page, expect

# Constants
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
@pytest.mark.int_id("INT-172")
def test_int_172_plan_approval_control_placement(page: Page):
    """
    INT-172: Approve/disapprove controls are available in both expected UI locations
    (chat-bottom and file-explorer/top-right) when planning is complete; controls are
    hidden/disabled before planner output is ready.
    """
    # 1. Start a benchmark generation
    # NOTE: Direct goto('/benchmark') doesn't work because http-server --proxy
    # proxies non-static paths to the controller. Navigate via React Router instead.
    page.goto(FRONTEND_URL)
    page.wait_for_load_state("networkidle")
    page.evaluate("localStorage.clear()")
    page.reload()
    page.wait_for_load_state("networkidle")

    expect(page.get_by_test_id("chat-confirm-button")).not_to_be_visible()
    expect(
        page.locator("[data-testid='file-explorer-confirm-button']")
    ).not_to_be_visible()
    expect(page.get_by_role("button", name="Planning")).not_to_be_visible()

    page.get_by_role("link", name="Benchmark").click()
    expect(page).to_have_url(re.compile(r".*/benchmark"))

    # Click CREATE NEW
    page.get_by_test_id("create-new-button").click()

    prompt_text = f"Generate a simple benchmark for approval test {uuid.uuid4()}"
    page.locator("#chat-input").fill(prompt_text)

    # Verify controls are hidden/disabled before planner output is ready
    chat_confirm_button = page.get_by_test_id("chat-confirm-button")
    # We will just look for Confirm & Start in chat
    expect(chat_confirm_button).not_to_be_visible()

    file_explorer_confirm_button = page.locator(
        "[data-testid='file-explorer-confirm-button']"
    )
    expect(file_explorer_confirm_button).not_to_be_visible()

    # Send message to start planning
    page.get_by_label("Send Message").click()

    # 2. Wait for the planner to finish (indicator: status changes to PLANNED)
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return data.episodeStatus === 'PLANNED';
            } catch (e) { return false; }
        }""",
        timeout=180000,
    )

    # 3. Assert Approve/disapprove controls are available in chat-bottom
    expect(chat_confirm_button).to_be_visible(timeout=10000)

    # 4. Assert Approve/disapprove controls are available in file-explorer/top-right
    # Assuming there's a file explorer confirm button per the spec
    expect(file_explorer_confirm_button).to_be_visible(timeout=10000)
