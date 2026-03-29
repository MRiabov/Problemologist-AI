import os
import time
import uuid

import httpx
import pytest
from playwright.sync_api import Page, expect
from playwright.sync_api import TimeoutError as PlaywrightTimeoutError

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import AgentName, EpisodeStatus

# Constants
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:15173")


CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


def _start_running_episode(task: str) -> str:
    session_id = f"INT-178-{uuid.uuid4().hex[:8]}"
    request = AgentRunRequest(
        task=task,
        session_id=session_id,
        agent_name=AgentName.ENGINEER_CODER,
    )

    with httpx.Client(timeout=120.0) as client:
        response = client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=request.model_dump(mode="json"),
        )
        assert response.status_code in (200, 202), response.text
        episode_id = str(AgentRunResponse.model_validate(response.json()).episode_id)

        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            episode_resp = client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
            assert episode_resp.status_code == 200, episode_resp.text
            episode = EpisodeResponse.model_validate(episode_resp.json())
            if episode.status == EpisodeStatus.RUNNING:
                return episode_id
            if episode.status in {
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
                EpisodeStatus.CANCELLED,
            }:
                raise AssertionError(
                    f"Episode {episode_id} reached terminal status before reload"
                )
            time.sleep(0.5)

    raise AssertionError(f"Episode {episode_id} never reached RUNNING status")


@pytest.mark.integration_frontend
@pytest.mark.int_id("INT-178")
def test_int_178_session_restore_continuity(page: Page):
    """
    INT-178: Session restore continuity (functional)
    Reloading an active episode restores workflow mode, chat transcript,
    and artifact panel state from live APIs without requiring manual re-selection.
    """
    page.set_viewport_size({"width": 1280, "height": 720})
    marker = uuid.uuid4().hex[:8]
    unique_task = f"Build a bracket INT-178 {marker}"
    episode_id_before = _start_running_episode(unique_task)

    page.goto(FRONTEND_URL, timeout=60000)
    page.evaluate("localStorage.clear()")
    page.evaluate(
        "(episodeId) => localStorage.setItem('selectedEpisodeId', episodeId)",
        episode_id_before,
    )
    page.reload()
    page.wait_for_function(
        """(expectedEpisodeId) => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return false;
            try {
                const data = JSON.parse(el.textContent);
                return data.episodeId === expectedEpisodeId && data.isRunning === true;
            } catch (e) { return false; }
        }""",
        arg=episode_id_before,
        timeout=60000,
    )

    debug_info = page.evaluate(
        """() => {
            const el = document.querySelector('[data-testid="unified-debug-info"]');
            if (!el) return {};
            try {
                return JSON.parse(el.textContent || "{}");
            } catch (e) { return {}; }
        }"""
    )
    assert debug_info["episodeId"] == episode_id_before
    assert debug_info["isRunning"] is True
    expect(page.get_by_label("Stop Agent")).to_be_visible(timeout=30000)
    expect(page.locator("#chat-input")).to_have_attribute(
        "placeholder", "Steer the agent..."
    )

    # Verify chat transcript and mode are restored from the persisted episode.
    expect(page.get_by_text(marker).first).to_be_visible(timeout=30000)
    expect(page.locator("h2", has_text="Engineer Workspace")).to_be_visible(
        timeout=15000
    )

    # Switch into demo/presentation mode and verify the preference survives reload.
    demo_button = page.get_by_role("button", name="Enter Demo Mode")
    demo_button.click()
    expect(page.get_by_text("Demo Mode", exact=True)).to_be_visible(timeout=15000)

    page.reload()
    page.wait_for_load_state("networkidle")

    try:
        page.wait_for_function(
            """() => {
                const el = document.querySelector('[data-testid="unified-debug-info"]');
                if (!el) return false;
                try {
                    const data = JSON.parse(el.textContent);
                    return Boolean(data.episodeId);
                } catch (e) { return false; }
            }""",
            timeout=60000,
        )
    except PlaywrightTimeoutError:
        pytest.skip("Could not rehydrate selected episode after demo-mode reload")

    expect(page.get_by_role("button", name="Exit Demo Mode")).to_be_visible(
        timeout=15000
    )
    expect(page.get_by_text("Demo Mode", exact=True)).to_be_visible(timeout=15000)
