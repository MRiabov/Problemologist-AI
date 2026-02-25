import pytest
from playwright.sync_api import Page, expect


@pytest.mark.integration_frontend
def test_simulation_navigation_timeline(page: Page):
    # 1. Navigate to the local development server
    page.goto("http://localhost:15173", timeout=60000)

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
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=120000)

    # Ensure Viewport overlays are gone before proceeding
    expect(page.get_by_text("No Assets Loaded")).not_to_be_visible(timeout=30000)
    expect(page.get_by_text("No Model Loaded")).not_to_be_visible(timeout=30000)
    expect(page.get_by_role("button", name="Rebuild Assets")).not_to_be_visible(
        timeout=30000
    )

    # 8. Test Simulation Controls
    # Play button
    play_button = page.locator("button:has(svg.lucide-play)")
    expect(play_button).to_be_visible(timeout=30000)
    play_button.click()

    # After click, it should change to Pause
    pause_button = page.locator("button:has(svg.lucide-pause)")
    expect(pause_button).to_be_visible(timeout=5000)

    # Timeline slider
    timeline_slider = page.locator("input[type='range']")
    expect(timeline_slider).to_be_visible()

    # Set slider to 50%
    timeline_slider.fill("50")
    expect(timeline_slider).to_have_value("50")

    # Verify time display (should show 5.0s if 50% of 10s)
    time_display = page.get_by_text("5.0s")
    expect(time_display).to_be_visible()

    # Test Reset (RotateCcw icon)
    reset_button = page.locator(
        "button:has(svg.lucide-rotate-ccw)"
    ).first  # There might be two, one for camera one for timeline
    # The one in the footer is usually grouped with rewind/fastforward
    footer_controls = page.locator(".bg-card\\/80")
    footer_reset = footer_controls.locator("button:has(svg.lucide-rotate-ccw)")
    footer_reset.click()

    # Slider should go back to 0
    expect(timeline_slider).to_have_value("0")
    expect(page.get_by_text("0.0s")).to_be_visible()
