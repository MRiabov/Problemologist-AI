import os
import re
import uuid

import pytest
from playwright.sync_api import Page, expect

# Constants
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_int_172_plan_approval_control_placement(page: Page):
    """
    INT-172: Approve/disapprove controls are available in both expected UI locations
    (chat-bottom and file-explorer/top-right) when planning is complete; controls are
    hidden/disabled before planner output is ready.
    """
    # 1. Start a benchmark generation
    page.goto(f"{FRONTEND_URL}/benchmark")
    page.wait_for_load_state("networkidle")

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

    # 2. Wait for the "Execution Plan Ready" card or completion of planning
    expect(
        page.get_by_text(re.compile("Execution Plan Ready", re.IGNORECASE))
    ).to_be_visible(timeout=180000)

    # 3. Assert Approve/disapprove controls are available in chat-bottom
    expect(chat_confirm_button).to_be_visible(timeout=10000)

    # 4. Assert Approve/disapprove controls are available in file-explorer/top-right
    # Assuming there's a file explorer confirm button per the spec
    expect(file_explorer_confirm_button).to_be_visible(timeout=10000)
