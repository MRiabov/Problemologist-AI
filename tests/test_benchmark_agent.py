from unittest.mock import MagicMock, AsyncMock, patch
import pytest
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from src.generators.benchmark.agent import generator_agent, DEFAULT_RUNTIME_CONFIG


@pytest.mark.benchmark
@pytest.mark.asyncio
async def test_benchmark_agent_validation_loop():
    """
    Test the benchmark agent's ability to retry upon validation failure.
    """
    # 1. Planner Mocks
    planner_mock = MagicMock()
    planner_mock.ainvoke = AsyncMock(side_effect=[
        AIMessage(content="Plan: Create a box."),
        AIMessage(content="Plan already exists and is valid. Proceeding to execution.")
    ])

    # 2. Actor Mocks
    actor_mock = MagicMock()
    actor_mock.bind_tools.return_value = actor_mock
    actor_mock.ainvoke = AsyncMock(side_effect=[
        AIMessage(
            content="Attempt 1",
            tool_calls=[
                {
                    "name": "validate_benchmark_model",
                    "args": {"code": "invalid"},
                    "id": "call_1",
                    "type": "tool_call",
                }
            ],
        ),
        AIMessage(
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
    ])

    # 3. Critic Mocks
    critic_mock = MagicMock()
    critic_mock.ainvoke = AsyncMock(side_effect=[
        AIMessage(content="Validation Failed. Please fix the code."),
        AIMessage(content="Validation Passed! Task complete.")
    ])

    def get_mock_model(node_type):
        if "planner" in node_type: return planner_mock
        if "actor" in node_type: return actor_mock
        if "critic" in node_type: return critic_mock
        return MagicMock()

    with (
        patch("src.agent.graph.nodes.planner.get_model", return_value=planner_mock),
        patch("src.agent.graph.nodes.actor.get_model", return_value=actor_mock),
        patch("src.agent.graph.nodes.critic.get_model", return_value=critic_mock),
    ):
        with (
            patch(
                "src.generators.benchmark.manager.execute_build", return_value="<xml/>"
            ),
            patch("src.generators.benchmark.validator.validate_mjcf") as mock_val,
        ):
            val_counters = {"v": 0}

            def val_side_effect(xml, asset_dir=None):
                val_counters["v"] += 1
                if val_counters["v"] == 1:
                    return {"is_valid": False, "error_message": "Err"}
                return {"is_valid": True, "error_message": None}

            mock_val.side_effect = val_side_effect

            inputs = {
                "messages": [HumanMessage(content="Box")],
                "plan": "",
                "step_count": 0,
                "runtime_config": DEFAULT_RUNTIME_CONFIG,
            }

            result = await generator_agent.ainvoke(
                inputs, config={"recursion_limit": 50}
            )

            # Check for tool messages in the final messages list
            tool_msgs = [m for m in result["messages"] if isinstance(m, ToolMessage)]
            assert len(tool_msgs) >= 2


@pytest.mark.benchmark
@pytest.mark.asyncio
async def test_benchmark_agent_success():
    """
    Test one-shot success.
    """
    planner_mock = MagicMock()
    planner_mock.ainvoke = AsyncMock(return_value=AIMessage(content="Plan"))

    actor_mock = MagicMock()
    actor_mock.bind_tools.return_value = actor_mock
    actor_mock.ainvoke = AsyncMock(return_value=AIMessage(
        content="Code",
        tool_calls=[
            {
                "name": "validate_benchmark_model",
                "args": {"code": "c"},
                "id": "t1",
                "type": "tool_call",
            }
        ],
    ))

    critic_mock = MagicMock()
    critic_mock.ainvoke = AsyncMock(return_value=AIMessage(content="Validation Passed! Task complete."))

    with (
        patch("src.agent.graph.nodes.planner.get_model", return_value=planner_mock),
        patch("src.agent.graph.nodes.actor.get_model", return_value=actor_mock),
        patch("src.agent.graph.nodes.critic.get_model", return_value=critic_mock),
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
                {"messages": [HumanMessage(content="X")], "plan": "", "step_count": 0, "runtime_config": DEFAULT_RUNTIME_CONFIG}
            )
            tool_msgs = [m for m in result["messages"] if isinstance(m, ToolMessage)]
            assert len(tool_msgs) == 1
            assert "Validation Passed" in str(tool_msgs[0].content)