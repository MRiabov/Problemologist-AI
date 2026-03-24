import os
import re
import time
import uuid

import httpx
import pytest
from playwright.sync_api import Page, expect

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


def _wait_for_terminal_status(page: Page) -> None:
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return ['COMPLETED', 'FAILED', 'CANCELLED', 'PLANNED'].includes(data.episodeStatus);
            } catch (e) { return false; }
        }""",
        timeout=180000,
    )


def _find_episode_id_by_task(task: str) -> str:
    with httpx.Client(timeout=10.0) as client:
        for _ in range(45):
            resp = client.get(f"{CONTROLLER_URL}/api/episodes/")
            assert resp.status_code == 200
            for ep in resp.json():
                if ep.get("task") == task:
                    return ep["id"]
            time.sleep(1)
    raise AssertionError(f"Episode not found for task: {task}")


@pytest.mark.integration_frontend
def test_int_161_tool_activity_and_reasoning_visibility(page: Page):
    """
    INT-161: Run real tool calls and assert activity feed entries originate from backend traces.
    Also validates reasoning visibility when "View reasoning" is enabled.
    """

    def handle_page_error(exc):
        print(f"\nPAGEERROR: {exc}")

    def handle_console(msg):
        if msg.type in ["error", "warning"]:
            print(f"\nCONSOLE[{msg.type}]: {msg.text}")

    page.on("pageerror", handle_page_error)
    page.on("console", handle_console)

    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.reload()
    page.wait_for_load_state("networkidle")

    # Prefer benchmark route for richer planner/coder traces, but continue on workspace if nav is unavailable.
    benchmark_link = page.get_by_role("link", name="Benchmark")
    try:
        expect(benchmark_link).to_be_visible(timeout=10000)
        benchmark_link.click()
        expect(page).to_have_url(re.compile(r".*/benchmark"))
    except Exception:
        print(
            "\nDEBUG: Benchmark nav unavailable; continuing from current workflow page."
        )

    create_btn = page.get_by_test_id("create-new-button")
    expect(create_btn).to_be_visible(timeout=30000)
    create_btn.click()
    unique_task = f"INT-161 tool/reasoning visibility {uuid.uuid4()}"
    chat_input = page.locator("#chat-input")
    chat_input.fill(unique_task)
    page.get_by_label("Send Message").click()

    # If planner pauses for confirmation, continue execution.
    try:
        page.wait_for_selector('[data-testid="chat-confirm-button"]', timeout=30000)
        page.get_by_test_id("chat-confirm-button").click(force=True)
    except Exception:
        pass

    _wait_for_terminal_status(page)

    episode_id = _find_episode_id_by_task(unique_task)
    with httpx.Client(timeout=10.0) as client:
        ep_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert ep_resp.status_code == 200
        traces = ep_resp.json().get("traces", [])

    tool_traces = [t for t in traces if t.get("trace_type") == "TOOL_START"]
    assert tool_traces, "Expected at least one TOOL_START trace from backend."

    reasoning_traces = [
        t
        for t in traces
        if t.get("trace_type") == "LLM_END" and t.get("name") and t.get("content")
    ]
    assert reasoning_traces, (
        "Expected backend reasoning traces (LLM_END with node name)."
    )

    log_traces = [
        t
        for t in traces
        if t.get("trace_type") == "LOG"
        and (t.get("metadata_vars") or {}).get("role") != "user"
        and not str(t.get("content") or "").startswith("User message:")
    ]
    assert log_traces, "Expected persisted non-user LOG traces from the backend."

    # Verify tool activity rows are rendered in UI from backend traces.
    activity_rows = page.locator('[data-testid="tool-activity-row"]')
    expect(activity_rows.first).to_be_visible(timeout=30000)
    assert activity_rows.count() >= len(tool_traces), (
        f"UI tool rows ({activity_rows.count()}) < backend tool traces ({len(tool_traces)})."
    )

    first_log_content = str(log_traces[0].get("content") or "").strip()
    first_log_snippet = first_log_content.splitlines()[0][:80]
    assert first_log_snippet, "Expected a non-user LOG trace with persisted content."
    expect(
        page.get_by_test_id("run-log-row").filter(has_text=first_log_snippet).first
    ).to_be_visible(timeout=30000)

    # Verify reasoning is hidden by default then shown when enabled.
    toggle = page.get_by_test_id("view-reasoning-toggle")
    expect(toggle).to_have_text(re.compile(r"Off"))
    expect(page.locator('[data-testid="reasoning-span"]')).to_have_count(0)

    toggle.click()
    expect(toggle).to_have_text(re.compile(r"On"))

    reasoning_spans = page.locator('[data-testid="reasoning-span"]')
    expect(reasoning_spans.first).to_be_visible(timeout=30000)

    # Expand a few spans and ensure at least one backend reasoning snippet is visible.
    expected_snippet = (
        (reasoning_traces[0]["content"] or "").strip().splitlines()[0][:40]
    )
    assert expected_snippet, "Backend reasoning trace content was empty."

    max_expand = min(reasoning_spans.count(), 5)
    for i in range(max_expand):
        reasoning_spans.nth(i).click()

    expect(
        page.get_by_text(re.compile(re.escape(expected_snippet))).first
    ).to_be_visible(timeout=30000)
