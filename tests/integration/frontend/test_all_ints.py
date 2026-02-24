import pytest
from playwright.sync_api import Page, expect

FRONTEND_URL = "http://localhost:5173/"

@pytest.mark.integration_playwright
@pytest.mark.integration_p0
def test_int_167_worker_cad_fetch(page: Page):
    """INT-167: Worker-only CAD fetch with security contract."""
    requests = []
    def handle_request(request):
        if "/api/worker/assets/" in request.url:
            requests.append({
                "url": request.url,
                "method": request.method,
                "headers": request.headers
            })
    page.on("request", handle_request)
    page.goto(FRONTEND_URL)
    chat_input = page.locator("#chat-input")
    chat_input.click()
    chat_input.fill("INT-167")
    page.get_by_label("Send Message").click()
    try:
        glb_button = page.locator("button:has-text('test_model.glb')")
        # Give more time for polling and rendering
        print("Waiting for test_model.glb button...")
        expect(glb_button).to_be_visible(timeout=120000)
        glb_button.click()
        print("Found and clicked test_model.glb")
    except Exception as e:
        print(f"Failed to find test_model.glb. All buttons: {page.locator('button').all_text_contents()}")
        page.screenshot(path="debug_int_167_failure.png")
        raise e

    page.wait_for_timeout(5000)
    worker_gets = [r for r in requests if "/api/worker/assets/" in r["url"] and r["method"] == "GET"]
    assert len(worker_gets) > 0, "No GET requests to worker assets found"

    # Check for security contract: ensure session ID is present and GET only
    for req in worker_gets:
        assert "sessionId=" in req["url"], f"Session ID missing in URL: {req['url']}"

    # Verify that a non-GET request to worker proxy is blocked (security contract)
    # We can use page.evaluate to try a POST
    blocked = page.evaluate("""async () => {
        const resp = await fetch('/api/worker/assets/test.glb?sessionId=test', { method: 'POST' });
        return resp.status;
    }""")
    assert blocked == 405, f"POST to worker proxy should be blocked (405), got {blocked}"

@pytest.mark.integration_playwright
@pytest.mark.integration_p1
def test_int_168_circuit_viewer(page: Page):
    """INT-168: Circuit viewer integration."""
    page.goto(FRONTEND_URL)
    prompt = "INT-168"
    chat_input = page.locator("#chat-input")
    chat_input.click()
    chat_input.fill(prompt)
    page.get_by_label("Send Message").click()

    # Wait for completion and check for assembly definition
    assy_btn = page.locator("button:has-text('assembly_definition.yaml')")
    print("Waiting for assembly_definition.yaml button...")
    expect(assy_btn).to_be_visible(timeout=120000)
    assy_btn.click()
    print("Found and clicked assembly_definition.yaml")

    # Toggle electronics view (Zap icon)
    zap_btn = page.get_by_title("Toggle Electronics View")
    expect(zap_btn).to_be_visible(timeout=10000)
    zap_btn.click()

    # Check for schematic button in ArtifactView (it shows when electronics field is present)
    try:
        schematic_tab = page.locator("button:has-text('Schematic')")
        expect(schematic_tab).to_be_visible()
        schematic_tab.click()
    except Exception as e:
        print(f"All buttons: {page.locator('button').all_text_contents()}")
        page.screenshot(path="debug_int_168_failure.png")
        raise e

    # Verify schematic canvas/container
    expect(page.locator(".tscircuit-container, canvas")).to_be_visible()

@pytest.mark.integration_playwright
@pytest.mark.integration_p2
def test_int_169_theme_persistence(page: Page):
    """INT-169: Theme preference persistence."""
    page.goto(FRONTEND_URL)

    # Switch to dark theme
    settings_btn = page.get_by_label("Settings")
    settings_btn.click()

    dark_btn = page.locator("button:has-text('Dark')")
    dark_btn.click()

    # Verify dark class on html or body
    expect(page.locator("html")).to_have_class("dark")

    # Reload page
    page.reload()

    # Verify theme persists
    expect(page.locator("html")).to_have_class("dark")

    # Switch back to light
    light_btn = page.locator("button:has-text('Light')")
    light_btn.click()
    expect(page.locator("html")).not_to_have_class("dark")
