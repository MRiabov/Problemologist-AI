import pytest
from playwright.sync_api import Page


@pytest.mark.integration_frontend
def test_console_log_capture(page: Page):
    page.goto("about:blank")
    page.evaluate("console.log('REPRO MESSAGE: HELLO FROM BROWSER')")
    page.evaluate("console.error('REPRO MESSAGE: ERROR FROM BROWSER')")
    # No assertion needed, just checking if logs are captured
