from playwright.sync_api import Page, expect
import pytest


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
    prompt_input = page.get_by_placeholder("Describe the benchmark goal...")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)
    print("Prompt filled.")

    # 5. Submit the prompt
    print("Submitting prompt...")
    prompt_input.press("Enter")
    print("Prompt submitted.")

    # 6. Wait for the generation to start and a plan to appear
    print("Waiting for Reasoning Trace...")
    expect(page.get_by_text("Reasoning Trace")).to_be_visible(timeout=60000)
    print("Reasoning Trace visible.")

    # Now look for the plan in ArtifactView.
    print("Waiting for plan.md tab...")
    # The plan.md button is in the Explorer sidebar
    # We use .first because there might be multiple matches in the UI
    plan_button = page.get_by_role("button", name="plan.md").first
    expect(plan_button).to_be_visible(timeout=120000)
    plan_button.click()
    print("Clicked plan.md button.")

    # 7. Verify the plan is "non-template"
    print("Verifying plan content...")
    # Based on ArtifactView.tsx, content is in a font-mono pre tag
    plan_content = page.locator("pre.whitespace-pre-wrap")
    expect(plan_content).to_be_visible(timeout=30000)

    # Verify it mentions "steel ball" or "40mm"
    expect(plan_content).to_contain_text("steel ball", ignore_case=True)
    expect(plan_content).to_contain_text("40mm", ignore_case=True)
    print("Verified keywords in plan.")

    # Final check: ensure "goal" is mentioned
    expect(plan_content).to_contain_text("goal", ignore_case=True)
    print("Final check passed.")

    print("Success: Benchmark creation flow verified.")
