import pytest
from playwright.sync_api import Page, expect
import time

@pytest.mark.integration_playwright
def test_int_171_layout_persistence(page: Page):
    """
    INT-171: 3-column layout + resize persistence
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto("http://localhost:5173", timeout=60000)
    # Clear localStorage to ensure a clean state for the test
    page.evaluate("localStorage.clear()")
    page.reload()

    # Wait for app to load
    page.wait_for_selector('[data-testid="app-layout"]', timeout=30000)

    # Check default sidebar ratio (expected ~0.25)
    sidebar = page.locator('[data-panel]').nth(0)
    window_width = 1280
    sidebar_box = sidebar.bounding_box()
    assert sidebar_box is not None
    sidebar_ratio = sidebar_box['width'] / window_width
    assert 0.20 < sidebar_ratio < 0.35, f"Sidebar ratio {sidebar_ratio} is not approx 0.25"

    # Resize Sidebar
    handle = page.locator("[data-panel-resize-handle]").first
    handle_box = handle.bounding_box()
    assert handle_box is not None

    # Drag handle to the right by 100px
    page.mouse.move(handle_box['x'] + handle_box['width']/2, handle_box['y'] + handle_box['height']/2)
    page.mouse.down()
    page.mouse.move(handle_box['x'] + 100, handle_box['y'] + handle_box['height']/2)
    page.mouse.up()

    time.sleep(1) # Wait for debounce/persistence

    new_sidebar_box = sidebar.bounding_box()
    assert new_sidebar_box is not None

    # Reload and verify persistence
    page.reload()
    page.wait_for_selector('[data-testid="app-layout"]', timeout=30000)

    persisted_sidebar_box = page.locator('[data-panel]').nth(0).bounding_box()
    assert persisted_sidebar_box is not None
    assert abs(persisted_sidebar_box['width'] - new_sidebar_box['width']) < 15


@pytest.mark.integration_playwright
def test_int_170_feedback_system(page: Page):
    """
    INT-170: Post-run feedback UX + API persistence
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    page.goto("http://localhost:5173", timeout=60000)
    page.evaluate("localStorage.clear()")
    page.reload()

    # 1. Start a session
    page.wait_for_selector('button:has-text("CREATE NEW")', timeout=30000)
    page.get_by_role("button", name="CREATE NEW").click()

    prompt_input = page.locator("#chat-input")
    prompt_input.wait_for(state="attached", timeout=30000)
    prompt_input.fill("Design a test part.")

    send_button = page.get_by_label("Send Message")
    send_button.click()

    # 2. Wait for generation to complete (ends in 'planned' status in mock benchmark mode)
    page.wait_for_selector('.lucide-check-circle2, .lucide-clock', timeout=60000)

    # 3. Verify Thumbs Up/Down icons appear in Sidebar on hover
    session_entry = page.locator("button.group").first
    session_entry.hover()

    thumbs_up = page.locator('[data-testid="sidebar-thumbs-up"]').first
    thumbs_up.wait_for(state="visible", timeout=10000)

    # 4. Click Thumbs Up and verify modal opens
    thumbs_up.click(force=True)
    expect(page.get_by_text("Agent Feedback", exact=True)).to_be_visible(timeout=10000)

    # 5. Fill feedback and submit
    page.get_by_role("button", name="Instruction Following").click()
    page.get_by_placeholder("What was satisfying about this response?").fill("Great job!")

    submit_button = page.get_by_role("button", name="Send Feedback")
    expect(submit_button).to_be_enabled()
    submit_button.click()

    # 6. Verify success state
    expect(page.get_by_text("Feedback Received")).to_be_visible(timeout=30000)
