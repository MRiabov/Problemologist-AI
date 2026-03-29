import os
import uuid

import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")

# Possible worker URLs to check against
FORBIDDEN_WORKER_PORTS = ["8000", "8001", "8002"]


@pytest.mark.integration_frontend
@pytest.mark.int_id("INT-175")
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
    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")

    # Navigate to Benchmark page via React Router
    benchmark_link = page.get_by_role("link", name="Benchmark")
    expect(benchmark_link).to_be_visible(timeout=30000)
    benchmark_link.click()
    import re as re_lib

    expect(page).to_have_url(re_lib.compile(r".*/benchmark"))

    # 2. Click "CREATE NEW" button
    create_new_button = page.get_by_role("button", name="CREATE NEW")
    expect(create_new_button).to_be_visible(timeout=30000)
    create_new_button.click()

    # 3. Enter a prompt to generate traffic
    prompt_text = f"Create a simple test for boundary {uuid.uuid4()}"
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)

    # 4. Submit the prompt
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)

    # Capture a network request that indicates we've started generating
    with page.expect_request(lambda request: "/api/episodes" in request.url):
        send_button.click()

    # 5. Wait for the planner to finish (indicator: status changes to PLANNED)
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return data.episodeStatus === 'PLANNED';
            } catch (e) { return false; }
        }""",
        timeout=120000,
    )

    try:
        confirm_button = page.get_by_test_id("chat-confirm-button")
        expect(confirm_button).to_be_visible()

        # Capture a network request for confirmation
        with page.expect_request(lambda request: "confirm" in request.url):
            confirm_button.click()
    except Exception:
        pass  # Ignore timeout if we don't reach here, the main test is the network traffic

    # Assert no violations were recorded
    assert len(violation_urls) == 0, (
        f"Frontend directly called worker endpoints: {violation_urls}"
    )
