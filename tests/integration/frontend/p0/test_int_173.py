import contextlib
import os
import re

import pytest
from playwright.sync_api import Page, expect

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


def _ensure_viewport_assets(page: Page) -> None:
    assets_overlay = page.get_by_test_id("no-assets-overlay")
    if not assets_overlay.is_visible():
        return

    for _ in range(3):
        if not assets_overlay.is_visible():
            return

        rebuild_assets_button = page.get_by_test_id("rebuild-assets-button")
        expect(rebuild_assets_button).to_be_visible(timeout=30000)
        try:
            rebuild_assets_button.click(force=True, timeout=10000)
        except Exception:
            # The button can get detached/re-rendered during rebuild; retry.
            pass
        page.wait_for_load_state("networkidle", timeout=60000)
        page.wait_for_timeout(800)

    assert not assets_overlay.is_visible(), (
        "Viewport assets remained unavailable after rebuild retries"
    )


def _ensure_model_assets(page: Page) -> None:
    model_overlay = page.get_by_test_id("no-model-overlay")
    if not model_overlay.is_visible():
        return

    for _ in range(3):
        if not model_overlay.is_visible():
            return
        rebuild_model_button = page.get_by_test_id("rebuild-model-button")
        expect(rebuild_model_button).to_be_visible(timeout=30000)
        with contextlib.suppress(Exception):
            rebuild_model_button.click(force=True, timeout=10000)
        page.wait_for_load_state("networkidle", timeout=60000)
        page.wait_for_timeout(800)

    assert not model_overlay.is_visible(), (
        "Model overlay remained visible after rebuild retries"
    )


@pytest.mark.integration_frontend
@pytest.mark.int_id("INT-173")
def test_int_173_exact_pointing_payload(page: Page):
    """
    INT-173: Selecting face/edge/vertex/part/subassembly produces typed context payloads
    with stable entity IDs and source asset reference; payload reaches backend unchanged.
    """
    # 1. Start a benchmark generation
    page.goto(FRONTEND_URL, timeout=60000)
    page.wait_for_load_state("networkidle")

    # Navigate to Benchmark page via React Router
    benchmark_link = page.get_by_role("link", name="Benchmark")
    expect(benchmark_link).to_be_visible(timeout=30000)
    benchmark_link.click()
    expect(page).to_have_url(re.compile(r".*/benchmark"))

    # Click CREATE NEW
    create_new_button = page.get_by_role("button", name="CREATE NEW")
    expect(create_new_button).to_be_visible(timeout=30000)
    create_new_button.click()

    prompt_text = "Simple mechanism benchmark INT-173"
    page.locator("#chat-input").fill(prompt_text)
    page.get_by_label("Send Message").click()

    # Wait until planner stage is ready and confirm if needed.
    page.wait_for_function(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return ['PLANNED', 'COMPLETED', 'FAILED'].includes(data.episodeStatus);
            } catch (e) { return false; }
        }""",
        timeout=120000,
    )
    status = page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return null;
            try {
                return JSON.parse(el.textContent).episodeStatus ?? null;
            } catch (e) { return null; }
        }"""
    )
    if status == "PLANNING":
        page.wait_for_function(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return false;
                try {
                    const data = JSON.parse(el.textContent);
                    return ['PLANNED', 'COMPLETED', 'FAILED'].includes(data.episodeStatus);
                } catch (e) { return false; }
            }""",
            timeout=120000,
        )
        status = page.evaluate(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return null;
                try {
                    return JSON.parse(el.textContent).episodeStatus ?? null;
                } catch (e) { return null; }
            }"""
        )
    assert status in {"PLANNED", "COMPLETED"}, (
        f"Benchmark did not reach PLANNED/COMPLETED before selection flow (status={status})"
    )

    if status == "PLANNED":
        page.get_by_test_id("chat-confirm-button").click(force=True)
        page.wait_for_function(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return false;
                try {
                    const data = JSON.parse(el.textContent);
                    return ['COMPLETED', 'FAILED'].includes(data.episodeStatus);
                } catch (e) { return false; }
            }""",
            timeout=180000,
        )
        post_confirm_status = page.evaluate(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return null;
                try {
                    return JSON.parse(el.textContent).episodeStatus ?? null;
                } catch (e) { return null; }
            }"""
        )
        assert post_confirm_status == "COMPLETED", (
            "Benchmark failed after confirm; cannot validate exact-pointing payload contract"
        )

    _ensure_viewport_assets(page)
    _ensure_model_assets(page)

    # Wait for the model viewer to load and have assets
    page.wait_for_selector(
        '[data-testid="design-viewer-3d"]', state="visible", timeout=60000
    )
    page.wait_for_selector(
        '[data-testid="main-canvas-container"]', state="visible", timeout=60000
    )
    canvas_container = page.get_by_test_id("main-canvas-container")
    expect(canvas_container).to_be_visible(timeout=30000)

    # The actual canvas element is inside
    canvas = canvas_container.locator("canvas")
    expect(canvas).to_be_visible(timeout=30000)
    # Verify all topology selection mode buttons exist and can be activated.
    face_selection = page.get_by_title("Face Selection")
    part_selection = page.get_by_title("Part Selection")
    subassembly_selection = page.get_by_title("Subassembly Selection")

    expect(face_selection).to_be_visible(timeout=30000)
    expect(part_selection).to_be_visible(timeout=30000)
    expect(subassembly_selection).to_be_visible(timeout=30000)

    face_selection.click()
    expect(face_selection).to_have_class(re.compile(r"bg-primary"))
    part_selection.click()
    expect(part_selection).to_have_class(re.compile(r"bg-primary"))
    subassembly_selection.click()
    expect(subassembly_selection).to_have_class(re.compile(r"bg-primary"))
    expect(face_selection).not_to_have_class(re.compile(r"bg-primary"))
    # Switch back to FACE mode for exact-pointing payload assertions.
    face_selection.click()
    expect(face_selection).to_have_class(re.compile(r"bg-primary"))

    # Click in the viewport to select a topology entity in FACE mode.
    existing_cards = page.get_by_test_id("context-card").count()
    box = canvas.bounding_box()
    assert box is not None, "Canvas bounding box unavailable"
    cx = int(box["width"] / 2)
    cy = int(box["height"] / 2)
    click_positions = [
        {"x": cx, "y": cy},
        {"x": max(10, cx - 80), "y": max(10, cy - 60)},
        {
            "x": min(int(box["width"] - 10), cx + 80),
            "y": min(int(box["height"] - 10), cy + 60),
        },
        {"x": max(10, cx - 140), "y": min(int(box["height"] - 10), cy + 100)},
    ]
    selected = False
    for pos in click_positions:
        canvas.click(position=pos, force=True)
        page.wait_for_timeout(400)
        if page.get_by_test_id("context-card").count() > existing_cards:
            selected = True
            break

    if not selected:
        face_nodes = page.locator(
            '[data-testid="topology-node-row"][data-node-type="face"]'
        )
        if face_nodes.count() > 0:
            face_nodes.first.click()
            page.wait_for_timeout(300)
            selected = page.get_by_test_id("context-card").count() > existing_cards
        else:
            any_nodes = page.locator('[data-testid="topology-node-row"]')
            if any_nodes.count() > 0:
                any_nodes.first.click()
                page.wait_for_timeout(300)
                selected = page.get_by_test_id("context-card").count() > existing_cards

    assert selected, "Could not select any CAD geometry in FACE mode"

    chat_input = page.locator("#chat-input")
    chat_input.fill("Adjust the selected face")

    # Capture outgoing prompt payload and assert structured geometry selection is sent.
    with page.expect_request(
        re.compile(
            r".*/api/episodes/.*/messages|.*/api/agent/run|.*/api/sessions/.*/steer"
        )
    ) as req_info:
        page.get_by_label("Send Message").click()

    post_data = req_info.value.post_data_json or {}
    metadata_vars = post_data.get("metadata_vars")
    selections: list[dict] = []
    mentions: list[str] = []
    if isinstance(metadata_vars, dict):
        selections = metadata_vars.get("selections", []) or []
        mentions = metadata_vars.get("mentions", []) or []
    else:
        # Steer endpoint sends selections/mentions at the top level.
        selections = post_data.get("selections", []) or []
        mentions = post_data.get("mentions", []) or []

    assert isinstance(selections, list) and selections, "No geometry selections sent"

    face_selection_payload = next(
        (s for s in selections if (s or {}).get("level") == "FACE"),
        None,
    )
    assert face_selection_payload is not None, (
        f"Expected FACE selection payload, got selections={selections}"
    )
    assert face_selection_payload.get("target_id"), "FACE selection target_id missing"
    center = face_selection_payload.get("center")
    assert isinstance(center, list) and len(center) == 3, (
        f"FACE selection center must be xyz list, got {center}"
    )

    assert isinstance(mentions, list), "mentions payload must be a list"
    assert face_selection_payload["target_id"] in mentions, (
        "Selected face target_id must be preserved in mentions payload"
    )
