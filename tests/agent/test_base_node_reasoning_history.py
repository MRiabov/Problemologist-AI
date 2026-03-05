from types import SimpleNamespace
from unittest.mock import patch

from controller.agent.nodes.base import BaseNode
from shared.enums import AgentName


class _DummyRecorder:
    def __init__(self) -> None:
        self.calls: list[dict[str, str]] = []

    def record_reasoning_text_sync(
        self,
        *,
        node_name: str,
        reasoning_text: str,
        step_index: int | None = None,
        source: str = "live_tool_loop",
    ) -> None:
        self.calls.append(
            {
                "node_name": node_name,
                "reasoning_text": reasoning_text,
                "source": source,
                "step_index": str(step_index),
            }
        )


def test_log_lm_history_delta_emits_reasoning_from_string_and_dict_outputs() -> None:
    history = [
        {
            "outputs": [
                "Thought: inspect objectives first\nAction: read_file(path='/objectives.yaml')",
                {"reasoning": "Now list config directory"},
                {
                    "text": "[[ ## next_thought ## ]]\nRead skills first\n[[ ## next_tool_name ## ]]\nread_file",
                    "reasoning_content": "Internal long reasoning block",
                },
            ]
        }
    ]
    ctx = SimpleNamespace(
        dspy_lm=SimpleNamespace(history=history),
        session_id="s-1",
        episode_id=None,
    )
    node = BaseNode(context=ctx)
    recorder = _DummyRecorder()

    node._log_lm_history_delta(
        node_type=AgentName.ENGINEER_PLANNER,
        attempt=1,
        history_start_idx=0,
        db_callback=recorder,  # type: ignore[arg-type]
    )

    emitted = [c["reasoning_text"] for c in recorder.calls]
    assert "inspect objectives first" in emitted
    assert "Now list config directory" in emitted
    assert "Read skills first" in emitted
    assert "Internal long reasoning block" in emitted
    assert all(c["source"] == "lm_history" for c in recorder.calls)


def test_log_lm_history_delta_reports_langfuse_usage() -> None:
    history = [
        {
            "outputs": ["Thought: do task"],
            "usage": {"prompt_tokens": 10, "completion_tokens": 5},
            "model": "openai/gpt-5-mini",
            "cost": 0.0001,
        }
    ]
    ctx = SimpleNamespace(
        dspy_lm=SimpleNamespace(history=history),
        session_id="s-1",
        episode_id=None,
    )
    node = BaseNode(context=ctx)

    with patch(
        "controller.agent.nodes.base.report_usage_to_current_observation"
    ) as mock_report:
        node._log_lm_history_delta(
            node_type=AgentName.ENGINEER_PLANNER,
            attempt=1,
            history_start_idx=0,
            db_callback=None,
        )

    mock_report.assert_called_once_with(
        usage={"prompt_tokens": 10, "completion_tokens": 5},
        model="openai/gpt-5-mini",
        cost=0.0001,
    )
