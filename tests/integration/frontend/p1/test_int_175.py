import os
import uuid

import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")

# Possible worker URLs to check against (including integration test ports)
FORBIDDEN_WORKER_PORTS = ["8000", "8001", "8002", "18001", "18002"]


@pytest.mark.integration_frontend
def test_int_175_controller_first_api_boundary(page: Page):
    """
    INT-175: Browser network traffic for chat/files/assets/sessions uses
    controller endpoints only; frontend never calls worker host directly
    in normal operation.
    """
    violation_urls = []

    def handle_request(request):
        url = request.url
        # Ignore data URIs and non-http requests
        if not url.startswith("http"):
            return

        # Ignore Vite/React HMR requests if any (they go to the frontend port)
        if str(FRONTEND_URL) in url:
            return

        # Check if URL hits any forbidden worker ports
        for port in FORBIDDEN_WORKER_PORTS:
            if f":{port}/" in url or f":{port}" == url[-len(f":{port}") :]:
                violation_urls.append(url)

    page.on("request", handle_request)

    # 1. Navigate to the local development server
    page.goto(f"{FRONTEND_URL}/benchmark", timeout=60000)
    page.wait_for_load_state("networkidle")

    # 2. Click "CREATE NEW" button
    page.get_by_test_id("create-new-button").click()

    # 3. Enter a prompt to generate traffic
    prompt_text = f"Create a simple test for boundary {uuid.uuid4()}"
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)

    # 4. Submit the prompt
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)
    send_button.click()

    # Let some traffic happen
    page.wait_for_timeout(5000)

    # 5. Wait for the "Confirm & Start" button and click it to trigger more traffic
    try:
        confirm_button = page.get_by_role("button", name="Confirm & Start")
        expect(confirm_button).to_be_visible(timeout=120000)
        confirm_button.click()
        page.wait_for_timeout(5000)
    except Exception:
        pass  # Ignore timeout if we don't reach here, the main test is the network traffic

    # Assert no violations were recorded
    assert len(violation_urls) == 0, (
        f"Frontend directly called worker endpoints: {violation_urls}"
    )
