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
    confirm_button = page.get_by_role("button", name="Confirm & Start")
    expect(confirm_button).to_be_visible(timeout=120000)
    confirm_button.click()

    # 7. Wait for assets to be generated (Send Message button returns)
    # Note: We expect the 3D viewer to load after this.
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=120000)

    # Ensure Viewport overlays are gone before proceeding
    expect(page.get_by_text("No Assets Loaded")).not_to_be_visible(timeout=30000)
    expect(page.get_by_text("No Model Loaded")).not_to_be_visible(timeout=30000)
    expect(page.get_by_role("button", name="Rebuild Assets")).not_to_be_visible(
        timeout=30000
    )

    # 8. Test Topology Browser Toggle
    topology_toggle = page.get_by_title("Toggle Model Browser")
    expect(topology_toggle).to_be_visible(timeout=30000)

    # Check if Model Browser is visible (it should be by default)
    model_browser = page.locator(".w-72.shrink-0.z-20")  # ModelBrowser class
    expect(model_browser).to_be_visible(timeout=30000)

    # Toggle it off
    topology_toggle.click()
    expect(model_browser).not_to_be_visible(timeout=30000)

    # Toggle it back on
    topology_toggle.click()
    expect(model_browser).to_be_visible(timeout=30000)

    # 9. Test Selection Modes
    face_selection = page.get_by_title("Face Selection")
    part_selection = page.get_by_title("Part Selection")
    subassembly_selection = page.get_by_title("Subassembly Selection")

    expect(face_selection).to_be_visible()
    expect(part_selection).to_be_visible()
    expect(subassembly_selection).to_be_visible()

    # Click Face Selection
    face_selection.click()
    # It should have the bg-primary class (or equivalent indicating active state)
    # Since it's a ghost button that gets bg-primary when active:
    expect(face_selection).to_have_class(re.compile(r"bg-primary"))

    # Click Subassembly Selection
    subassembly_selection.click()
    expect(subassembly_selection).to_have_class(re.compile(r"bg-primary"))
    expect(face_selection).not_to_have_class(re.compile(r"bg-primary"))
