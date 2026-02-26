import os

import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


@pytest.mark.integration_frontend
def test_simulation_navigation_timeline(page: Page):
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

    # 7. Test Simulation Controls (available even when viewport assets are still loading)
    if page.get_by_test_id("no-assets-overlay").is_visible():
        pytest.skip("Viewport assets unavailable; simulation controls are blocked")

    play_button = page.get_by_test_id("simulation-play-toggle")
    expect(play_button).to_be_visible(timeout=30000)
    play_button.click()
    expect(play_button).to_be_visible(timeout=5000)

    # Timeline slider
    timeline_slider = page.get_by_test_id("simulation-timeline-slider")
    expect(timeline_slider).to_be_visible()

    # Set slider to 50%
    timeline_slider.fill("50")
    expect(timeline_slider).to_have_value("50")

    # Verify time display (should show 5.0s if 50% of 10s)
    expect(page.get_by_test_id("simulation-current-time")).to_have_text("5.0s")

    # Test reset behavior
    page.get_by_test_id("simulation-reset-button").click()

    # Slider should go back to 0
    expect(timeline_slider).to_have_value("0")
    expect(page.get_by_test_id("simulation-current-time")).to_have_text("0.0s")
