from unittest.mock import MagicMock, patch
import pytest
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
from src.generators.benchmark.agent import generator_agent


@pytest.mark.benchmark
@pytest.mark.asyncio
async def test_benchmark_agent_validation_loop():
    """
    Test the benchmark agent's ability to retry upon validation failure.
    """
    mock_llm = MagicMock()
    mock_llm.bind_tools.return_value = mock_llm

    call_count = {"actor": 0}

    def mock_invoke(messages, **kwargs):
        system_msg = next(
            (m.content for m in messages if isinstance(m, SystemMessage)), ""
        )

        if "planner" in system_msg:
            return AIMessage(content="Plan: Create a box.")

        if "coder" in system_msg or "build123d" in system_msg:
            call_count["actor"] += 1
            if call_count["actor"] == 1:
                return AIMessage(
                    content="Attempt 1",
                    tool_calls=[
                        {
                            "name": "validate_benchmark_model",
                            "args": {"code": "invalid"},
                            "id": "call_1",
                            "type": "tool_call",
                        }
                    ],
                )
            elif call_count["actor"] == 2:
                return AIMessage(
                    content="Attempt 2",
                    tool_calls=[
                        {
                            "name": "validate_benchmark_model",
                            "args": {"code": "valid"},
                            "id": "call_2",
                            "type": "tool_call",
                        }
                    ],
                )
            else:
                return AIMessage(content="Success message")

        if "critic" in system_msg:
            return AIMessage(content="Verified")

        return AIMessage(content="Default")

    mock_llm.invoke.side_effect = mock_invoke

    with (
        patch("src.agent.graph.nodes.planner.get_model", return_value=mock_llm),
        patch("src.agent.graph.nodes.actor.get_model", return_value=mock_llm),
        patch("src.agent.graph.nodes.critic.get_model", return_value=mock_llm),
    ):
        with (
            patch(
                "src.generators.benchmark.manager.execute_build", return_value="<xml/>"
            ),
            patch("src.generators.benchmark.validator.validate_mjcf") as mock_val,
        ):
            val_counters = {"v": 0}

            def val_side_effect(xml):
                val_counters["v"] += 1
                if val_counters["v"] == 1:
                    return {"is_valid": False, "error_message": "Err"}
                return {"is_valid": True, "error_message": None}

            mock_val.side_effect = val_side_effect

            inputs = {
                "messages": [HumanMessage(content="Box")],
                "plan": "",
                "step_count": 0,
                "runtime_config": {},
            }

            result = await generator_agent.ainvoke(
                inputs, config={"recursion_limit": 20}
            )

            # Basic checks
            assert call_count["actor"] >= 2

            # Check for tool messages
            tool_msgs = [m for m in result["messages"] if hasattr(m, "tool_call_id")]
            assert len(tool_msgs) >= 2
            assert "Validation Failed" in tool_msgs[0].content
            assert "Validation Passed" in tool_msgs[1].content


@pytest.mark.benchmark
@pytest.mark.asyncio
async def test_benchmark_agent_success():
    """
    Test one-shot success.
    """
    mock_llm = MagicMock()
    mock_llm.bind_tools.return_value = mock_llm

    def mock_invoke(messages, **kwargs):
        system_msg = next(
            (m.content for m in messages if isinstance(m, SystemMessage)), ""
        )
        if "planner" in system_msg:
            return AIMessage(content="Plan")
        if "coder" in system_msg:
            return AIMessage(
                content="Code",
                tool_calls=[
                    {
                        "name": "validate_benchmark_model",
                        "args": {"code": "c"},
                        "id": "t1",
                        "type": "tool_call",
                    }
                ],
            )
        return AIMessage(content="Verified")

    mock_llm.invoke.side_effect = mock_invoke

    with (
        patch("src.agent.graph.nodes.planner.get_model", return_value=mock_llm),
        patch("src.agent.graph.nodes.actor.get_model", return_value=mock_llm),
        patch("src.agent.graph.nodes.critic.get_model", return_value=mock_llm),
    ):
        with (
            patch(
                "src.generators.benchmark.manager.execute_build", return_value="<xml/>"
            ),
            patch(
                "src.generators.benchmark.validator.validate_mjcf",
                return_value={"is_valid": True, "error_message": None},
            ),
        ):
            result = await generator_agent.ainvoke(
                {"messages": [HumanMessage(content="X")], "plan": "", "step_count": 0}
            )
            tool_msgs = [m for m in result["messages"] if hasattr(m, "tool_call_id")]
            assert len(tool_msgs) == 1
            assert "Validation Passed" in tool_msgs[0].content
