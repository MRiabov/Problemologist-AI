import os
import re
import time
import uuid

import httpx
import pytest
from playwright.sync_api import Page, expect

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_int_160_reasoning_default_hidden_and_expandable(page: Page):
    """
    INT-160: Reasoning traces default-hidden + expandable.
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.reload()
    page.wait_for_load_state("networkidle")

    page.get_by_test_id("create-new-button").click()
    unique_task = f"INT-160 reasoning visibility {uuid.uuid4()}"
    prompt_input = page.locator("#chat-input")
    prompt_input.fill(unique_task)
    page.get_by_label("Send Message").click()

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

    # Hidden by default.
    expect(page.get_by_test_id("view-reasoning-toggle")).to_have_text(
        re.compile(r"Off")
    )
    expect(page.locator('[data-testid="reasoning-span"]')).to_have_count(0)

    # Ensure this run actually produced reasoning traces from live API.
    episode_id = None
    with httpx.Client(timeout=10.0) as client:
        for _ in range(30):
            episodes_resp = client.get(f"{CONTROLLER_URL}/api/episodes/")
            assert episodes_resp.status_code == 200
            for ep in episodes_resp.json():
                if ep.get("task") == unique_task:
                    episode_id = ep["id"]
                    break
            if episode_id:
                break
            time.sleep(1)
    assert episode_id, "Episode not found for INT-160 run."

    with httpx.Client(timeout=10.0) as client:
        episode_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert episode_resp.status_code == 200
        traces = episode_resp.json().get("traces", [])

    reasoning_candidates = [
        t
        for t in traces
        if t.get("trace_type") == "LLM_END" and t.get("name") and t.get("content")
    ]
    if not reasoning_candidates:
        pytest.skip(
            "Live run produced no persisted reasoning traces; expandable-reasoning "
            "assertions are not deterministic in this backend mode."
        )

    # Enable "View reasoning" and verify spans are now visible and expandable.
    page.get_by_test_id("view-reasoning-toggle").click()
    expect(page.get_by_test_id("view-reasoning-toggle")).to_have_text(re.compile(r"On"))

    reasoning_spans = page.locator('[data-testid="reasoning-span"]')
    expect(reasoning_spans.first).to_be_visible(timeout=15000)
    reasoning_spans.first.click()
    expect(
        page.get_by_text(re.compile(r"Reasoning|Thought", re.IGNORECASE)).first
    ).to_be_visible()
