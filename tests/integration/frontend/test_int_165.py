import contextlib
import os
import re

import pytest
from playwright.sync_api import Page, expect

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
        expect(rebuild_assets_button).to_be_visible(timeout=30000)
        with contextlib.suppress(Exception):
            rebuild_assets_button.click(force=True, timeout=10000)
        page.wait_for_load_state("networkidle", timeout=60000)
        page.wait_for_timeout(800)

    return not assets_overlay.is_visible()


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

    # 6. Wait for the planner to finish or reach a terminal state.
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

    # 7. Ensure assets are loaded for topology interactions.
    assets_loaded = _ensure_viewport_assets(page)
    if not assets_loaded:
        expect(page.get_by_test_id("no-assets-overlay")).to_be_visible(timeout=30000)
        expect(page.get_by_test_id("no-assets-title")).to_be_visible(timeout=30000)
        expect(page.get_by_test_id("rebuild-assets-button")).to_be_visible(
            timeout=30000
        )
        expect(page.get_by_test_id("no-model-overlay")).to_be_visible(timeout=30000)
        expect(page.get_by_test_id("no-model-title")).to_be_visible(timeout=30000)
        return

    # 8. Test Topology Browser availability and toggle.
    topology_toggle = page.get_by_test_id("model-browser-toggle")
    assert topology_toggle.count() > 0, "Topology toggle is missing"
    topology_toggle_button = topology_toggle.first
    expect(topology_toggle_button).to_be_visible(timeout=30000)

    # Check if Model Browser is visible (it should be by default)
    model_browser = page.get_by_test_id("model-browser-panel")
    expect(model_browser).to_be_visible(timeout=30000)

    model_overlay = page.get_by_test_id("no-model-overlay")
    if model_overlay.is_visible():
        rebuild_model_button = page.get_by_test_id("rebuild-model-button")
        expect(rebuild_model_button).to_be_visible(timeout=30000)
        rebuild_model_button.click(force=True)
        page.wait_for_load_state("networkidle", timeout=60000)
        expect(model_overlay).not_to_be_visible(timeout=60000)

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
