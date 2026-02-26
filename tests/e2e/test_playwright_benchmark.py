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

    # 7. Wait for the generation to finish (Send Message button returns)
    # This might take a while, increase timeout
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=120000)

    # 8. Select plan.md in Resources sidebar
    plan_button = page.get_by_role("button", name="plan.md")
    expect(plan_button).to_be_visible(timeout=30000)
    plan_button.click()

    # 9. Verify the plan is "non-template"
    # The content is rendered in a SyntaxHighlighter viewport
    content_area = page.locator(".flex-1.min-w-0.bg-background\\/50")
    expect(
        content_area.get_by_text(re.compile("steel ball", re.IGNORECASE))
    ).to_be_visible(timeout=30000)
    expect(content_area.get_by_text(re.compile("40mm", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )

    # Final check: ensure "goal" is mentioned
    expect(content_area.get_by_text(re.compile("goal", re.IGNORECASE))).to_be_visible(
        timeout=30000
    )
