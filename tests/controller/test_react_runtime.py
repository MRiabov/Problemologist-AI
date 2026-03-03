import dspy

from controller.agent.react_runtime import FirstTurnFullInputReAct


def test_first_turn_full_input_then_compacted_followups():
    def noop() -> str:
        return "ok"

    react = FirstTurnFullInputReAct("task -> answer", tools=[noop], max_iters=4)
    long_task = "lift the ball " * 200

    seen_tasks: list[str] = []
    react_calls = {"count": 0}

    def fake_call(module, trajectory, **input_args):
        if module is react.react:
            seen_tasks.append(input_args["task"])
            if react_calls["count"] == 0:
                react_calls["count"] += 1
                return dspy.Prediction(
                    next_thought="read context",
                    next_tool_name="noop",
                    next_tool_args={},
                )
            react_calls["count"] += 1
            return dspy.Prediction(
                next_thought="done",
                next_tool_name="finish",
                next_tool_args={},
            )

        return dspy.Prediction(answer="final")

    react._call_with_potential_trajectory_truncation = fake_call  # type: ignore[method-assign]

    out = react(task=long_task)

    assert out.answer == "final"
    assert len(seen_tasks) >= 2
    assert seen_tasks[0] == long_task
    assert seen_tasks[1].startswith("[context elided after first turn:")
