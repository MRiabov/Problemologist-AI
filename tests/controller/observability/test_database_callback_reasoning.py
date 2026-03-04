import uuid

from controller.observability.database import DatabaseCallbackHandler


def _new_handler() -> DatabaseCallbackHandler:
    return DatabaseCallbackHandler(episode_id=str(uuid.uuid4()))


def test_extract_reasoning_steps_prefers_trajectory_then_summary() -> None:
    handler = _new_handler()
    output_obj = {
        "trajectory": {
            "thought_0": "Inspect objectives first",
            "tool_name_0": "read_file",
            "tool_args_0": {"path": "objectives.yaml"},
            "observation_0": "loaded constraints",
        },
        "reasoning": "Final summary",
    }

    steps = handler._extract_reasoning_steps(output_data="", output_obj=output_obj)
    assert len(steps) == 1
    assert steps[0].reasoning_step_index == 0
    assert steps[0].observation == "Inspect objectives first"


def test_extract_reasoning_steps_uses_reasoning_details_when_present() -> None:
    handler = _new_handler()
    output_obj = {
        "reasoning_details": [
            {"type": "reasoning.text", "text": "step detail 1", "index": 0},
            {"type": "reasoning.text", "text": "step detail 2", "index": 1},
        ]
    }

    steps = handler._extract_reasoning_steps(output_data="", output_obj=output_obj)
    assert len(steps) == 2
    assert steps[0].reasoning_step_index == 0
    assert steps[1].reasoning_step_index == 1
    assert steps[0].observation == "step detail 1"
    assert steps[1].observation == "step detail 2"


def test_extract_reasoning_steps_falls_back_to_string_parse() -> None:
    handler = _new_handler()
    output_data = "Prediction(trajectory={'thought_0': 'First thought'}, reasoning='Last summary')"

    steps = handler._extract_reasoning_steps(output_data=output_data, output_obj=None)
    assert len(steps) == 1
    assert steps[0].observation == "First thought"
