import re

import pytest
from playwright.sync_api import Page, expect

from tests.support.fixtures.factories.prompts import benchmark_prompt
from tests.support.page_objects.benchmark_workspace import BenchmarkWorkspacePage


@pytest.mark.integration_frontend
def test_benchmark_creation_flow(
    page: Page, benchmark_workspace: BenchmarkWorkspacePage
):
    benchmark_workspace.open()
    benchmark_workspace.open_benchmark()
    benchmark_workspace.create_new()

    prompt_text = benchmark_prompt(
        "We need to move a steel ball 40mm radius, 1m sideways. "
        "Start with a ball in one side of the simulation space and make "
        "a goal in the other. Make a benchmark that would test "
        "a machine that does it."
    )
    benchmark_workspace.submit_prompt(prompt_text)

    expect(page.get_by_label("Stop Agent")).to_be_visible(timeout=30000)

    assert (
        benchmark_workspace.wait_for_status(["PLANNED"], timeout_ms=120000) == "PLANNED"
    )
    benchmark_workspace.confirm_plan()
    assert (
        benchmark_workspace.wait_for_status(["COMPLETED"], timeout_ms=120000)
        == "COMPLETED"
    )
    expect(page.get_by_label("Send Message")).to_be_visible()

    benchmark_workspace.open_artifact("plan.md")
    plan_text = benchmark_workspace.code_view_text()

    assert re.search(
        r"learning objective|solution overview", plan_text, re.IGNORECASE
    ), f"plan.md missing expected structure. Content:\n{plan_text}"
    assert re.search(r"goal", plan_text, re.IGNORECASE), (
        f"plan.md missing goal context. Content:\n{plan_text}"
    )
    assert re.search(r"steel ball|sphere", plan_text, re.IGNORECASE), (
        f"plan.md missing moved object context. Content:\n{plan_text}"
    )
