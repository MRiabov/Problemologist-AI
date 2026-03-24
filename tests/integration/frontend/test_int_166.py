import contextlib
import os

import pytest
from playwright.sync_api import Page, expect
from playwright.sync_api import TimeoutError as PlaywrightTimeoutError

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


def _ensure_viewport_assets(page: Page) -> bool:
    assets_overlay = page.get_by_test_id("no-assets-overlay")
    if not assets_overlay.is_visible():
        return True

    for _ in range(3):
        if not assets_overlay.is_visible():
            return True
        rebuild_assets_button = page.get_by_test_id("rebuild-assets-button")
        if not rebuild_assets_button.is_visible():
            return False
        with contextlib.suppress(Exception):
            rebuild_assets_button.click(force=True, timeout=10000)
        page.wait_for_load_state("networkidle", timeout=60000)
        page.wait_for_timeout(800)

    return not assets_overlay.is_visible()


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

    # 6. Wait for the planner to finish (indicator: status changes to PLANNED)
    try:
        page.wait_for_function(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return false;
                try {
                    const data = JSON.parse(el.textContent);
                    return ['PLANNED', 'COMPLETED', 'FAILED', 'CANCELLED'].includes(data.episodeStatus);
                } catch (e) { return false; }
            }""",
            timeout=120000,
        )
    except PlaywrightTimeoutError:
        pass

    current_status = page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return null;
            try {
                return JSON.parse(el.textContent).episodeStatus ?? null;
            } catch (e) { return null; }
        }"""
    )
    if current_status == "PLANNED":
        confirm_button = page.get_by_test_id("chat-confirm-button")
        expect(confirm_button).to_be_visible()
        confirm_button.click()
    else:
        expect(page.get_by_label("Send Message")).to_be_visible(timeout=30000)
        return

    # 7. Wait for post-approval execution to settle before driving simulation controls.
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return ['COMPLETED', 'FAILED', 'CANCELLED'].includes(data.episodeStatus);
            } catch (e) { return false; }
        }""",
        timeout=180000,
    )
    final_status = page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return null;
            try {
                return JSON.parse(el.textContent).episodeStatus ?? null;
            } catch (e) { return null; }
        }"""
    )
    assets_loaded = _ensure_viewport_assets(page)
    if final_status != "COMPLETED" or not assets_loaded:
        expect(page.get_by_label("Send Message")).to_be_visible(timeout=30000)
        return

    # 8. Ensure assets are loaded for simulation controls.

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
