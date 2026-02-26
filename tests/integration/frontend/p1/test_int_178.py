import os
import uuid
import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_int_178_session_restore_continuity(page: Page):
    """
    INT-178: Session restore continuity (functional)
    Reloading an active episode restores workflow mode, chat transcript,
    and artifact panel state from live APIs without requiring manual re-selection.
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.reload()

    # 1. Start a session
    page.get_by_test_id("create-new-button").click()
    unique_task = f"Build a bracket {uuid.uuid4()}"
    prompt_input = page.locator("#chat-input")
    prompt_input.fill(unique_task)
    page.get_by_label("Send Message").click()

    # 2. Wait for some traces to appear (e.g., node start or thinking)
    # Using the thinking indicator as a sign of activity
    expect(page.get_by_text(re.compile(r"thinking", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )

    # 3. Get the episode ID from the UI or implicitly know it's selected
    # Verify the task name is in the sidebar and highlighted
    session_entry = page.get_by_test_id("sidebar-episode-item").filter(
        has_text=unique_task
    )
    expect(session_entry).to_be_visible()

    # Check if the session entry has the active (primary) class
    expect(session_entry).to_have_class(re.compile(r"bg-primary/10"))

    # 4. Reload the page
    page.reload()
    page.wait_for_load_state("networkidle")

    # 5. Verify the same session is still selected and highlighted
    # This check is crucial for INT-178
    session_entry_after = page.get_by_test_id("sidebar-episode-item").filter(
        has_text=unique_task
    )
    expect(session_entry_after).to_be_visible(timeout=30000)
    expect(session_entry_after).to_have_class(
        re.compile(r"bg-primary/10"), timeout=15000
    )

    # 6. Verify chat transcript is restored
    # At least the initial task should be there
    expect(page.get_by_text(unique_task).nth(1)).to_be_visible(
        timeout=15000
    )  # One in sidebar, one in chat

    # 7. Verify we are in the same workflow mode (Engineer Workspace)
    expect(page.locator("h2", has_text="Engineer Workspace")).to_be_visible(
        timeout=15000
    )


import re
