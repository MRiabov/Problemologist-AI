from __future__ import annotations

import json
from collections.abc import Sequence
from typing import Any

from playwright.sync_api import Page


def open_frontend(page: Page, frontend_url: str) -> None:
    page.goto(frontend_url, timeout=60000)
    page.wait_for_load_state("networkidle")


def get_debug_info(page: Page) -> dict[str, Any]:
    return page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return {};
            try {
                return JSON.parse(el.textContent || "{}");
            } catch (e) {
                return {};
            }
        }"""
    )


def wait_for_episode_status(
    page: Page,
    statuses: Sequence[str],
    *,
    timeout_ms: int = 180000,
) -> str | None:
    status_payload = json.dumps(list(statuses))
    page.wait_for_function(
        f"""() => {{
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {{
                const data = JSON.parse(el.textContent);
                return {status_payload}.includes(data.episodeStatus);
            }} catch (e) {{
                return false;
            }}
        }}""",
        timeout=timeout_ms,
    )
    return get_debug_info(page).get("episodeStatus")


def read_code_view_text(page: Page) -> str:
    code_lines = page.locator('[data-testid^="code-line-"]')
    return "\n".join(code_lines.all_inner_texts())
