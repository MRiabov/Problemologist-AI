import pytest
from playwright.sync_api import Page, expect


@pytest.mark.e2e
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

    # 6. Wait for the generation to start and a plan to appear
    print("Waiting for generation process...")

    # First, confirm the prompt appeared in the chat history
    # Scope to 'main' area to avoid matching historical sidebar items
    # Using a part of the prompt to avoid potential formatting/truncation issues
    expect(
        page.locator("main").get_by_text("move a steel ball 40mm radius", exact=False)
    ).to_be_visible(timeout=60000)
    print("Prompt confirmed in chat history.")

    # Now wait for the plan.md button to appear in the Resources sidebar
    print("Waiting for plan.md button...")
    plan_button = page.get_by_role("button", name="plan.md").first
    # The generation can take a while, especially with mock LLMs
    expect(plan_button).to_be_visible(timeout=120000)
    plan_button.click()
    print("Clicked plan.md button.")

    # 7. Verify the plan is "non-template"
    print("Verifying plan content...")
    # The content is rendered via SyntaxHighlighter
    # We can just look for the text in the artifact view area
    expect(page.get_by_text("steel ball", ignore_case=True)).to_be_visible(
        timeout=60000
    )
    expect(page.get_by_text("40mm", ignore_case=True)).to_be_visible(timeout=60000)
    print("Verified keywords in plan.")

    # Final check: ensure "goal" is mentioned
    expect(page.get_by_text("goal", ignore_case=True)).to_be_visible(timeout=60000)
    print("Final check passed.")

    print("Success: Benchmark creation flow verified.")
