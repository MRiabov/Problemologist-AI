from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass

from playwright.sync_api import Page, expect

from tests.support.helpers.playwright import (
    get_debug_info,
    open_frontend,
    read_code_view_text,
    wait_for_episode_status,
)


@dataclass(slots=True)
class BenchmarkWorkspacePage:
    page: Page
    frontend_url: str

    def open(self) -> None:
        open_frontend(self.page, self.frontend_url)

    def open_benchmark(self) -> None:
        benchmark_link = self.page.get_by_role(
            "link", name=re.compile(r"Benchmark", re.IGNORECASE)
        )
        if not benchmark_link.is_visible():
            benchmark_link = self.page.get_by_text("Benchmark").first

        expect(benchmark_link).to_be_visible(timeout=30000)
        benchmark_link.click()
        expect(self.page).to_have_url(re.compile(r".*/benchmark"), timeout=30000)

    def create_new(self) -> None:
        create_new_button = self.page.get_by_role("button", name="CREATE NEW")
        expect(create_new_button).to_be_visible(timeout=30000)
        create_new_button.click()

    def submit_prompt(self, prompt: str) -> None:
        prompt_input = self.page.locator("#chat-input")
        expect(prompt_input).to_be_visible(timeout=30000)
        prompt_input.fill(prompt)
        send_button = self.page.get_by_label("Send Message")
        expect(send_button).to_be_enabled(timeout=30000)
        send_button.click()

    def wait_for_status(
        self, statuses: Sequence[str], *, timeout_ms: int = 180000
    ) -> str | None:
        return wait_for_episode_status(self.page, statuses, timeout_ms=timeout_ms)

    def confirm_plan(self) -> None:
        confirm_button = self.page.get_by_test_id("chat-confirm-button")
        expect(confirm_button).to_be_visible(timeout=30000)
        confirm_button.click(force=True)

    def open_artifact(self, name: str) -> None:
        artifact_button = self.page.get_by_role("button", name=name).first
        expect(artifact_button).to_be_visible(timeout=30000)
        artifact_button.click(force=True)

    def code_view_text(self) -> str:
        return read_code_view_text(self.page)

    def debug_info(self) -> dict[str, object]:
        return get_debug_info(self.page)
