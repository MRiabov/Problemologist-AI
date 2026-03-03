import asyncio
import os
import uuid

import httpx
import pytest

from tests.integration.agent.helpers import (
    get_controller_log_path,
    read_log_segment,
    run_agent_episode,
    strip_ansi,
    wait_for_episode_terminal,
)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


def _extract_planner_full_texts(log_segment: str, session_id: str) -> list[str]:
    texts: list[str] = []
    expect_full_text = False

    for raw_line in log_segment.splitlines():
        line = strip_ansi(raw_line)

        if (
            "mock_dspy_lm_call" in line
            and f"session_id={session_id}" in line
            and "explicit_node=planner" in line
        ):
            expect_full_text = True
            continue

        if expect_full_text and "mock_dspy_full_text" in line:
            text_part = line.split("text=", 1)[1] if "text=" in line else line
            texts.append(text_part)
            expect_full_text = False
            continue

        if expect_full_text and "mock_dspy_lm_call" in line:
            expect_full_text = False

    return texts


@pytest.mark.integration_agent
@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_180_react_first_turn_full_context_followup_compaction():
    """INT-180: First turn keeps full context, follow-up tool turns compact it."""
    sentinel = f"INT180-SENTINEL-{uuid.uuid4().hex}"
    task = f"INT-180 prompt replay guard token: {sentinel}"

    log_path = get_controller_log_path()
    start_offset = log_path.stat().st_size if log_path.exists() else 0

    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id, episode_id = await run_agent_episode(
            client,
            int_id="INT-180",
            task=task,
            agent_name="engineer_coder",
        )

        episode = await wait_for_episode_terminal(client, episode_id)
        assert episode["status"] != "FAILED", f"Episode failed: {episode}"

    planner_texts: list[str] = []
    for _ in range(20):
        segment = read_log_segment(log_path, start_offset)
        planner_texts = _extract_planner_full_texts(segment, session_id)
        if len(planner_texts) >= 2:
            break
        await asyncio.sleep(0.5)

    assert len(planner_texts) >= 2, (
        "Expected at least two planner LLM turns in logs for compaction verification."
    )

    first_turn = planner_texts[0]
    followups = planner_texts[1:]

    assert sentinel in first_turn, (
        "Full sentinel context was missing in first LLM turn."
    )
    assert any(
        "[context elided after first turn: field=task" in text for text in followups
    ), "Expected compacted follow-up context marker after first tool turn."
    assert all(sentinel not in text for text in followups), (
        "Found full sentinel task replayed in follow-up turns after tool calls."
    )
