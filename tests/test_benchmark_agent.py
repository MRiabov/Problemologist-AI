from unittest.mock import MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from src.generators.benchmark.agent import generator_agent


@pytest.mark.benchmark
@pytest.mark.asyncio
async def test_generator_agent_mock():
    mock_llm = MagicMock()

    def mock_invoke(messages, **kwargs):
        all_content = " ".join([m.content for m in messages]).lower()
        if "physics puzzle designer" in all_content or "planner" in all_content:
            return AIMessage(
                content="<reasoning>I should create a box.</reasoning><plan>Plan: Create a box.</plan>"
            )
        if "build123d" in all_content or "coder" in all_content:
            return AIMessage(
                content='<reasoning>Implementing a box.</reasoning><python_code>```python\ndef build(seed=0):\n    return \'<mujoco><worldbody><geom type="box" size="0.1 0.1 0.1"/></worldbody></mujoco>\'\n```</python_code>'
            )
        return AIMessage(content="Default")

    mock_llm.bind_tools.return_value = mock_llm
    mock_llm.invoke.side_effect = mock_invoke

    with patch("src.generators.benchmark.agent.get_model", return_value=mock_llm):
        with patch("src.generators.benchmark.agent.run_linter", return_value=[]):
            with patch(
                "src.generators.benchmark.agent.validate_mjcf",
                return_value={"is_valid": True, "error_message": None},
            ):
                with patch(
                    "src.generators.benchmark.manager.execute_build",
                    return_value='<mujoco><worldbody><geom type="box" size="0.1 0.1 0.1"/></worldbody></mujoco>',
                ):
                    result = await generator_agent.ainvoke(
                        {"request": "Create a red box", "attempts": 0}
                    )
                    assert result["validation_passed"] is True
                    assert "mujoco" in result["mjcf"]


@pytest.mark.benchmark
@pytest.mark.asyncio
async def test_generator_agent_retry():
    mock_llm = MagicMock()

    call_count = {"coder": 0}

    def mock_invoke(messages, **kwargs):
        all_content = " ".join([m.content for m in messages]).lower()
        if "physics puzzle designer" in all_content or "planner" in all_content:
            return AIMessage(
                content="<reasoning>Thinking about box.</reasoning><plan>Plan: Create a box.</plan>"
            )
        elif "build123d" in all_content or "coder" in all_content:
            call_count["coder"] += 1
            if call_count["coder"] == 1:
                return AIMessage(
                    content="<reasoning>Fail first.</reasoning><python_code>```python\ndef build(seed=0):\n    return 'INVALID XML'\n```</python_code>"
                )
            return AIMessage(
                content="<reasoning>Succeed second.</reasoning><python_code>```python\ndef build(seed=0):\n    return '<mujoco><worldbody/></mujoco>'\n```</python_code>"
            )

        return AIMessage(content="Default")

    mock_llm.bind_tools.return_value = mock_llm
    mock_llm.invoke.side_effect = mock_invoke

    with patch("src.generators.benchmark.agent.get_model", return_value=mock_llm):
        with patch("src.generators.benchmark.agent.run_linter", return_value=[]):

            def mock_validate(xml, asset_dir=None):
                if xml == "INVALID XML":
                    return {"is_valid": False, "error_message": "Invalid XML"}
                return {"is_valid": True, "error_message": None}

            with patch(
                "src.generators.benchmark.agent.validate_mjcf",
                side_effect=mock_validate,
            ):

                def mock_execute_build(
                    code, seed, scale_factors=(1.0, 1.0, 1.0), **kwargs
                ):
                    if "INVALID XML" in code:
                        return "INVALID XML"
                    return "<mujoco><worldbody/></mujoco>"

                with patch(
                    "src.generators.benchmark.manager.execute_build",
                    side_effect=mock_execute_build,
                ):
                    result = await generator_agent.ainvoke(
                        {"request": "Create something", "attempts": 0}
                    )
                    assert result["validation_passed"] is True
                    assert result["attempts"] >= 2
