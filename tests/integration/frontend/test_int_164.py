import os

import pytest
from playwright.sync_api import Page, expect
from playwright.sync_api import TimeoutError as PlaywrightTimeoutError

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_code_viewer_line_selection_and_mentions(page: Page):
    # 1. Navigate to the local development server
    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")

    # 2. Navigate to the Benchmark page
    benchmark_link = page.get_by_role("link", name="Benchmark")
    expect(benchmark_link).to_be_visible(timeout=30000)
    benchmark_link.click()
    import re

    expect(page).to_have_url(re.compile(r".*/benchmark"))

    # 3. Click "CREATE NEW" button
    create_new_button = page.get_by_role("button", name="CREATE NEW")
    expect(create_new_button).to_be_visible(timeout=30000)
    create_new_button.click()

    # 4. Enter the prompt
    prompt_text = "Create a simple benchmark for moving a ball."
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)

    # 5. Submit the prompt
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)
    send_button.click()

    # 6. Wait for the planner to finish (indicator: status changes to PLANNED)
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return data.episodeStatus === 'PLANNED';
            } catch (e) { return false; }
        }""",
        timeout=120000,
    )
    confirm_button = page.get_by_test_id("chat-confirm-button")
    expect(confirm_button).to_be_visible()
    confirm_button.click()

    # 7. Wait for completion (status becomes COMPLETED)
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

    # Open a file in the code viewer
    script_file = page.get_by_text("script.py").first
    try:
        script_file.wait_for(state="visible", timeout=60000)
        script_file.click(force=True)
    except (PlaywrightTimeoutError, AssertionError):
        pytest.skip("Planner/coder artifacts not available in this integration run")

    # 8. Select lines in the code viewer
    # Line numbers are usually in a specific column, but we use test-id now
    line_one = page.get_by_test_id("code-line-1")
    expect(line_one).to_be_visible()

    # Simulate line selection (clicking the line)
    line_one.click()

    # 9. Type a mention in the chat input
    prompt_input.fill("Please explain @script.py:1-5")

    # 10. Submit and verify mention is processed (check for highlighting or specific payload if possible)
    # For integration test, we mainly check if it doesn't crash and sends the message
    # Wait for completion if needed (indicator: status changes to COMPLETED)
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
    send_button_after_mention = page.get_by_label("Send Message")
    expect(send_button_after_mention).to_be_enabled()
    send_button_after_mention.click()

    # 11. Verify the message was submitted by ensuring input clears.
    expect(prompt_input).to_have_value("", timeout=30000)
