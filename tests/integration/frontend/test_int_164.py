import pytest
from playwright.sync_api import Page, expect


@pytest.mark.integration_frontend
def test_code_viewer_line_selection_and_mentions(page: Page):
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
    prompt_text = "Create a simple benchmark for moving a ball."
    prompt_input = page.locator("#chat-input")
    expect(prompt_input).to_be_visible(timeout=30000)
    prompt_input.fill(prompt_text)

    # 5. Submit the prompt
    send_button = page.get_by_label("Send Message")
    expect(send_button).to_be_enabled(timeout=30000)
    send_button.click()

    # 6. Wait for the "Confirm & Start" button to appear and click it
    confirm_button = page.get_by_role("button", name="Confirm & Start")
    expect(confirm_button).to_be_visible(timeout=120000)
    confirm_button.click()

    # 7. Wait for assets to be generated (Send Message button returns)
    expect(page.get_by_label("Send Message")).to_be_visible(timeout=120000)

    # 8. Open a file in the code viewer
    # Clicking script.py in the file tree
    script_file = page.get_by_text("script.py")
    expect(script_file).to_be_visible()
    script_file.click()

    # 9. Select lines in the code viewer
    # Line numbers are usually in a specific column
    line_number = page.get_by_text("1", exact=True).first
    expect(line_number).to_be_visible()

    # Simulate line selection (clicking line number)
    line_number.click()

    # 10. Verify Context Card appears in chat input area
    context_card = page.locator(".bg-blue-500\\/10")  # Context card styling
    expect(context_card).to_be_visible()
    expect(context_card).to_contain_text("script.py")

    # 11. Type a mention in the chat input
    prompt_input.press_sequentially("Please explain @script.py:1-5")

    # 12. Submit and verify mention is processed (check for highlighting or specific payload if possible)
    # For integration test, we mainly check if it doesn't crash and sends the message
    send_button.click()

    # 13. Verify the message appears in chat with highlighted mention
    mention_highlight = page.locator(".text-blue-400.font-bold")
    expect(mention_highlight).to_be_visible()
