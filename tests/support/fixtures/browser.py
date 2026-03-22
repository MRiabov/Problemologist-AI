from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import pytest
from playwright.sync_api import Page

from tests.support.page_objects.benchmark_workspace import BenchmarkWorkspacePage

DEFAULT_FRONTEND_URL = "http://localhost:15173"
DEFAULT_CONTROLLER_URL = "http://localhost:18000"
DEFAULT_ARTIFACT_DIR = Path("test_output/playwright")


@pytest.fixture(scope="session")
def frontend_url() -> str:
    return os.getenv("BASE_URL") or os.getenv("FRONTEND_URL") or DEFAULT_FRONTEND_URL


@pytest.fixture(scope="session")
def controller_url() -> str:
    return os.getenv("API_URL") or os.getenv("CONTROLLER_URL") or DEFAULT_CONTROLLER_URL


@pytest.fixture(scope="session")
def playwright_artifact_dir() -> Path:
    path = Path(os.getenv("PLAYWRIGHT_ARTIFACTS_DIR", DEFAULT_ARTIFACT_DIR))
    path.mkdir(parents=True, exist_ok=True)
    return path


@pytest.fixture(scope="session")
def browser_context_args(
    browser_context_args: dict[str, Any], frontend_url: str
) -> dict[str, Any]:
    merged = dict(browser_context_args)
    merged.setdefault("base_url", frontend_url)
    return merged


@pytest.fixture
def benchmark_workspace(page: Page, frontend_url: str) -> BenchmarkWorkspacePage:
    return BenchmarkWorkspacePage(page=page, frontend_url=frontend_url)
