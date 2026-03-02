import re

import pytest
from playwright.sync_api import Page, expect


@pytest.mark.integration_frontend
def test_benchmark_creation_flow(page: Page):
    # 1. Navigate to the local development server
    page.goto("http://localhost:15173", timeout=60000)

    # 2. Navigate to the Benchmark page (using regex for flexibility with sidebar state)
    benchmark_link = page.get_by_role(
        "link", name=re.compile(r"Benchmark", re.IGNORECASE)
    )
    if not benchmark_link.is_visible():
        benchmark_link = page.get_by_text("Benchmark").first

    expect(benchmark_link).to_be_visible(timeout=30000)
    benchmark_link.click()

    # 3. Click "CREATE NEW" button
    create_new_button = page.get_by_role("button", name="CREATE NEW")
    expect(create_new_button).to_be_visible(timeout=30000)
    create_new_button.click()

    # 4. Enter the prompt
    prompt_text = (
        "We need to move a steel ball 40mm radius, 1m sideways. "
        "Start with a ball in one side of the simulation space and make "
        "a goal in the other. Make a benchmark that would test "
        "a machine that does it."
    )
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)

    # 5. Submit the prompt
    # Using explicit click on Send Message button for reliability
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)
    send_button.click()

    # 6. Wait for the generation to start (Stop Agent button appears)
    expect(page.get_by_label("Stop Agent")).to_be_visible(timeout=30000)

    # 7. Wait for the planner to finish (indicator: status changes to PLANNED)
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

    # 8. Wait for completion (status becomes COMPLETED)
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
    expect(page.get_by_label("Send Message")).to_be_visible()

    # 8. Select plan.md in Resources sidebar
    # Multiple "plan.md" entries can exist in different UI regions; use first resource match.
    plan_button = page.get_by_role("button", name="plan.md").first
    expect(plan_button).to_be_visible(timeout=30000)
    plan_button.click()

    # 9. Verify the plan is non-template and contains objective-specific language.
    # Use stable code-line test IDs instead of style-class selectors.
    first_line = page.get_by_test_id("code-line-1")
    expect(first_line).to_be_visible(timeout=30000)
    code_lines = page.locator('[data-testid^="code-line-"]')
    plan_text = "\n".join(code_lines.all_inner_texts())

    assert re.search(
        r"learning objective|solution overview", plan_text, re.IGNORECASE
    ), f"plan.md missing expected structure. Content:\n{plan_text}"
    assert re.search(r"goal", plan_text, re.IGNORECASE), (
        f"plan.md missing goal context. Content:\n{plan_text}"
    )
    assert re.search(r"steel ball|sphere", plan_text, re.IGNORECASE), (
        f"plan.md missing moved object context. Content:\n{plan_text}"
    )
