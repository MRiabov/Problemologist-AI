import pytest
from playwright.sync_api import Page, expect


@pytest.mark.integration_playwright
def test_benchmark_creation_flow(page: Page):
    print("\nStarting test_benchmark_creation_flow...")
    # 1. Navigate to the local development server
    print("Navigating to http://localhost:5173...")
    page.goto("http://localhost:5173", timeout=60000)
    print("Navigation successful.")

    # 2. Navigate to the Benchmark page
    print("Waiting for Benchmark link...")
    benchmark_link = page.get_by_role("link", name="Benchmark")
    expect(benchmark_link).to_be_visible(timeout=30000)
    benchmark_link.click()
    print("Clicked Benchmark link.")

    # 3. Click "CREATE NEW" button
    print("Waiting for CREATE NEW button...")
    create_new_button = page.get_by_role("button", name="CREATE NEW")
    expect(create_new_button).to_be_visible(timeout=30000)
    create_new_button.click()
    print("Clicked CREATE NEW button.")

    # 4. Enter the prompt
    print("Filling prompt...")
    prompt_text = (
        "We need to move a steel ball 40mm radius, 1m sideways. "
        "Start with a ball in one side of the simulation space and make "
        "a goal in the other. Make a benchmark that would test "
        "a machine that does it."
    )
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)
    print("Prompt filled.")

    # 5. Submit the prompt
    print("Submitting prompt...")
    # Using explicit click on Send Message button for reliability
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)
    send_button.click()
    print("Prompt submitted.")

    # 6. Wait for the generation to start (Stop Agent button appears)
    print("Waiting for agent to start...")
    expect(page.get_by_label("Stop Agent")).to_be_visible(timeout=30000)
    print("Agent started.")

    # 7. Wait for the generation to finish (Send Message button returns)
    print("Waiting for agent to finish...")
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=30000)
    print("Agent finished.")

    # 8. Select plan.md in Resources sidebar
    print("Selecting plan.md...")
    # The artifact sidebar has a "Resources" header
    sidebar = page.locator("div").filter(has_text="Resources").last
    plan_button = sidebar.get_by_role("button", name="plan.md")
    expect(plan_button).to_be_visible(timeout=30000)
    plan_button.click()
    print("Selected plan.md.")

    # 8. Verify the plan is "non-template"
    print("Verifying plan content...")
    # The content is rendered in a SyntaxHighlighter viewport
    content_area = page.locator(".flex-1.min-w-0.bg-background\\/50")
    expect(content_area.get_by_text("steel ball", ignore_case=True)).to_be_visible(
        timeout=30000
    )
    expect(content_area.get_by_text("40mm", ignore_case=True)).to_be_visible(
        timeout=30000
    )
    print("Verified keywords in plan.")

    # Final check: ensure "goal" is mentioned
    expect(content_area.get_by_text("goal", ignore_case=True)).to_be_visible(
        timeout=30000
    )
    print("Final check passed.")

    print("Success: Benchmark creation flow verified.")
