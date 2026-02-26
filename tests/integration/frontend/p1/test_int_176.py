import os
import uuid
import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_int_176_tool_call_failure_recovery(page: Page):
    """
    INT-176: Tool-call failure recovery path in chat
    Failed tool call message is rendered with failure reason,
    and subsequent successful tool calls/messages continue streaming without UI deadlock.
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.reload()

    # 1. Start a session with a specific ID to trigger the failure scenario
    # We use INT-176 as prefix for the session_id to match the scenario in mock_responses.yaml
    session_id = f"INT-176-{uuid.uuid4()}"

    # We need to hack the session_id generation in frontend or use a test endpoint
    # Actually, we can just use the prompt and hope it matches if we add a scenario for it.
    # But MockDSPyLM uses session_id.

    # Let's use the 'CREATE NEW' button and then steer it.
    page.get_by_role("button", name="CREATE NEW").click()

    # We'll use a prompt that we'll add to mock_responses.yaml
    unique_task = "Trigger a tool failure and recover INT-176"
    chat_input = page.locator("#chat-input")
    chat_input.fill(unique_task)

    # Force the session ID by intercepting the request or just let it be random
    # and rely on 'default' or a specific prompt match if I update MockDSPyLM.

    # Actually, I'll update mock_responses.yaml to have a scenario for this prompt.

    page.get_by_label("Send Message").click()

    # 2. Wait for the tool call to appear and show "Failed"
    # Based on our ActionCard.tsx change, it should show "Failed" with an AlertCircle
    failed_label = page.get_by_text("Failed")
    expect(failed_label).to_be_visible(timeout=60000)

    # 3. Verify that the agent continues and provides a final response
    # (Indicators: check-circle/clock/layers)
    page.wait_for_selector(
        ".lucide-check-circle2, .lucide-clock, .lucide-layers", timeout=60000
    )

    # 4. Verify chat is not deadlocked - we can send another message
    chat_input.fill("Now do something else")
    page.get_by_label("Send Message").click()

    # Wait for the user message to appear
    expect(
        page.get_by_test_id("chat-message").get_by_text("Now do something else")
    ).to_be_visible(timeout=30000)

    # Wait for agent to respond
    page.wait_for_selector(
        ".lucide-check-circle2, .lucide-clock, .lucide-layers", timeout=60000
    )


import re
