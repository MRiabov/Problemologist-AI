import pytest
import asyncio
from unittest.mock import MagicMock, patch
from langchain_core.messages import AIMessage
from src.generators.benchmark.agent import generator_agent


@pytest.mark.asyncio
async def test_generator_agent_mock():
    mock_llm = MagicMock()

    def mock_invoke(messages, **kwargs):
        all_content = " ".join([m.content for m in messages]).lower()
        if (
            "expert physics puzzle designer" in all_content
            or "detailed plan" in all_content
            or "planner" in all_content
        ):
            return AIMessage(content="Plan: Create a box.")
        elif (
            "build(seed: int = 0)" in all_content
            or "expert build123d coder" in all_content
            or "coder" in all_content
        ):
            return AIMessage(
                content='```python\ndef build(seed=0):\n    return \'<mujoco><worldbody><geom type="box" size="0.1 0.1 0.1"/></worldbody></mujoco>\'\n```'
            )
        return AIMessage(content="Default")

    mock_llm.invoke.side_effect = mock_invoke

    with patch("src.generators.benchmark.agent.get_model", return_value=mock_llm):
        with patch(
            "src.generators.benchmark.agent.validate_mjcf",
            return_value={"is_valid": True, "error_message": None},
        ):
            result = await generator_agent.ainvoke(
                {"request": "Create a red box", "attempts": 0}
            )
            assert result["validation_passed"] is True
            assert "mujoco" in result["mjcf"]


@pytest.mark.asyncio
async def test_generator_agent_retry():
    mock_llm = MagicMock()

    call_count = {"coder": 0}

    def mock_invoke(messages, **kwargs):
        all_content = " ".join([m.content for m in messages]).lower()
        if (
            "expert physics puzzle designer" in all_content
            or "detailed plan" in all_content
            or "planner" in all_content
        ):
            return AIMessage(content="Plan: Create a box.")
        elif (
            "build(seed: int = 0)" in all_content
            or "expert build123d coder" in all_content
            or "coder" in all_content
            or "fix the code" in all_content.lower()
        ):
            call_count["coder"] += 1
            # For test_generator_agent_retry
            if "invalid xml" in all_content and call_count["coder"] == 1:
                return AIMessage(
                    content="```python\ndef build(seed=0):\n    return 'INVALID XML'\n```"
                )

            if "invalid xml" in all_content:
                return AIMessage(
                    content="```python\ndef build(seed=0):\n    return '<mujoco><worldbody/></mujoco>'\n```"
                )

            # Default success for coder
            return AIMessage(
                content='```python\ndef build(seed=0):\n    return \'<mujoco><worldbody><geom type="box" size="0.1 0.1 0.1"/></worldbody></mujoco>\'\n```'
            )

        return AIMessage(content="Default")

    mock_llm.invoke.side_effect = mock_invoke

    with patch("src.generators.benchmark.agent.get_model", return_value=mock_llm):

        def mock_validate(xml):
            if xml == "INVALID XML":
                return {"is_valid": False, "error_message": "Invalid XML"}
            return {"is_valid": True, "error_message": None}

        with patch(
            "src.generators.benchmark.agent.validate_mjcf", side_effect=mock_validate
        ):
            result = await generator_agent.ainvoke(
                {"request": "Create something", "attempts": 0}
            )
            assert result["validation_passed"] is True
            assert result["attempts"] == 2
