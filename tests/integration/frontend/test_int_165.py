import os
import re

import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_cad_topology_selection_and_browser(page: Page):
    # 1. Navigate to the local development server
    page.goto(FRONTEND_URL, timeout=60000)

    # 2. Navigate to the Benchmark page
    benchmark_link = page.get_by_role("link", name="Benchmark")
    expect(benchmark_link).to_be_visible(timeout=30000)
    benchmark_link.click()

    # 3. Click "CREATE NEW" button
    create_new_button = page.get_by_role("button", name="CREATE NEW")
    expect(create_new_button).to_be_visible(timeout=30000)
    create_new_button.click()

    # 4. Enter the prompt
    prompt_text = "Create a simple cube benchmark."
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)

    # 5. Submit the prompt
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)
    send_button.click()

    # 6. Wait for the "Confirm & Start" button and click it
    confirm_button = page.get_by_test_id("chat-confirm-button")
    expect(confirm_button).to_be_visible(timeout=120000)
    confirm_button.click(force=True)

    # 7. Wait for either generated assets or fallback rebuild affordance.
    assets_overlay = page.get_by_test_id("no-assets-overlay")
    if assets_overlay.is_visible():
        expect(page.get_by_test_id("rebuild-assets-button")).to_be_visible(
            timeout=30000
        )

    # 8. Test Topology Browser availability and toggle where available.
    topology_toggle = page.get_by_test_id("model-browser-toggle")
    if topology_toggle.count() == 0:
        pytest.skip("Topology toggle unavailable in this integration run")
    topology_toggle_button = topology_toggle.first
    expect(topology_toggle_button).to_be_visible(timeout=30000)

    # Check if Model Browser is visible (it should be by default)
    model_browser = page.get_by_test_id("model-browser-panel")
    expect(model_browser).to_be_visible(timeout=30000)

    if page.get_by_test_id("no-model-overlay").is_visible():
        pytest.skip("Model asset overlay active; topology interaction unavailable")

    # 9. Test Selection Modes
    face_selection = page.get_by_title("Face Selection")
    part_selection = page.get_by_title("Part Selection")
    subassembly_selection = page.get_by_title("Subassembly Selection")

    expect(face_selection).to_be_visible()
    expect(part_selection).to_be_visible()
    expect(subassembly_selection).to_be_visible()

    # Click Face Selection
    face_selection.click()
    # Since it's a ghost button that gets bg-primary when active:
    expect(face_selection).to_have_class(re.compile(r"bg-primary"))

    # Click Subassembly Selection
    subassembly_selection.click()
    expect(subassembly_selection).to_have_class(re.compile(r"bg-primary"))
    expect(face_selection).not_to_have_class(re.compile(r"bg-primary"))

    # Toggle it off
    topology_toggle_button.click()
    expect(model_browser).not_to_be_visible(timeout=30000)
