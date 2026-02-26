import os
import uuid

import httpx
import pytest
from playwright.sync_api import Page, expect
from pydantic import TypeAdapter

from controller.api.schemas import EpisodeListItem, EpisodeResponse

# Constants
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_int_177_feedback_modal_edit_recall(page: Page):
    """
    INT-177: Feedback modal edit/recall + persistence contract
    After output completion, user can open feedback modal, change thumbs direction
    before submit, select topic(s), add text, and persisted feedback reflects final edited state.
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.reload()

    # 1. Start a session
    page.get_by_test_id("create-new-button").click()

    unique_prompt = f"Design a simple bracket INT-177 {uuid.uuid4()}"
    prompt_input = page.locator("#chat-input")
    prompt_input.wait_for(state="attached", timeout=30000)
    prompt_input.fill(unique_prompt)

    send_button = page.get_by_label("Send Message")
    send_button.click()

    # 2. Wait for generation to complete (indicators: check-circle/clock/layers/x-circle)
    # Using the same selectors as INT-170 + x-circle
    # We also wait for the status to change from RUNNING
    page.wait_for_function(
        "() => !document.querySelector('.animate-pulse')?.parentElement?.innerText?.includes('RUNNING')",
        timeout=120000,
    )

    page.wait_for_selector(
        ".lucide-check-circle2, .lucide-clock, .lucide-layers, .lucide-x-circle",
        timeout=120000,
    )

    # 3. Click Thumbs Up in chat
    # Need to hover the message first to make buttons visible
    # We look for any message content or thinking
    page.wait_for_selector(
        '[data-testid="chat-message"], [data-testid="thought-block"]', timeout=60000
    )

    # If it's a thought block, we might need to wait for the actual message
    page.wait_for_selector('[data-testid="chat-message"]', timeout=60000)

    last_message = page.get_by_test_id("chat-message").last
    last_message.hover()

    thumbs_up_chat = page.get_by_test_id("chat-thumbs-up").last
    # Sometimes hover takes a moment to trigger CSS transition
    for _ in range(5):
        if thumbs_up_chat.is_visible():
            break
        last_message.hover()
        page.wait_for_timeout(500)

    thumbs_up_chat.click()

    # 4. Verify modal opens and Thumbs Up is selected (score 1)
    modal = page.get_by_test_id("feedback-modal")
    expect(modal).to_be_visible(timeout=10000)

    # Check if Thumbs Up is active in modal.
    thumbs_up_modal = modal.get_by_test_id("modal-thumbs-up")
    expect(thumbs_up_modal).to_have_class(re.compile(r"text-green-500"))

    # 5. Click Thumbs Down in modal (change direction to score 0)
    thumbs_down_modal = modal.get_by_test_id("modal-thumbs-down")
    thumbs_down_modal.click()

    # Verify Thumbs Down is now active
    expect(thumbs_down_modal).to_have_class(re.compile(r"text-red-500"))

    # 6. Select a topic (e.g., "Technical Error")
    topic_name = "Technical Error"
    page.get_by_role("button", name=topic_name).click()

    # 7. Fill text and submit
    test_comment = f"Actually it had a technical error {uuid.uuid4()}"
    page.get_by_placeholder("How can the agent improve?").fill(test_comment)

    submit_button = page.get_by_role("button", name="Send Feedback")
    expect(submit_button).to_be_enabled()
    submit_button.click()

    # 8. Verify success state in UI
    expect(page.get_by_text("Feedback Received")).to_be_visible(timeout=30000)

    # 9. Verify via API that the final state (Thumbs Down + Comment) is persisted
    with httpx.Client() as client:
        # Get all episodes
        resp = client.get(f"{CONTROLLER_URL}/episodes/")
        assert resp.status_code == 200
        episodes = TypeAdapter(list[EpisodeListItem]).validate_python(resp.json())

        # The most recent episode should be ours
        # Episode names usually match the prompt or truncated prompt
        # But we can just check the last few episodes
        found_feedback = False
        for ep in episodes[:5]:  # Check first 5 episodes
            resp_ep = client.get(f"{CONTROLLER_URL}/episodes/{ep.id}")
            if resp_ep.status_code != 200:
                continue

            full_ep = EpisodeResponse.model_validate(resp_ep.json())
            # Find the trace that has feedback
            for trace in full_ep.traces:
                # Based on the implementation, feedback is likely attached to the assistant's message trace
                # In FeedbackSystem.tsx, submitTraceFeedback is called with traceId
                # Let's check if the feedback fields match
                if (
                    trace.feedback_score == 0
                    and trace.feedback_comment
                    and f"[{topic_name}] {test_comment}" in trace.feedback_comment
                ):
                    found_feedback = True
                    break
            if found_feedback:
                break

        assert found_feedback, (
            "Persisted feedback did not match final edited state (Score: 0, Comment: [Topic] Text)"
        )


import re
