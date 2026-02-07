import textwrap
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from worker.generators.benchmark.models import GenerationSession
from worker.generators.benchmark.nodes import coder_node, extract_python_code
from worker.generators.benchmark.state import BenchmarkGeneratorState


@pytest.fixture
def mock_state():
    session = GenerationSession(session_id=uuid4(), prompt="Create a simple bracket")
    return BenchmarkGeneratorState(
        session=session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        plan={"description": "A simple L-bracket"},
        messages=[],
    )


def test_extract_python_code():
    text = """Here is the code:
```python
print('hello')
```
Hope it helps!"""
    code = extract_python_code(text)
    assert code == "print('hello')"

    text_no_block = "print('world')"
    code_no_block = extract_python_code(text_no_block)
    assert code_no_block == "print('world')"


@pytest.mark.asyncio
async def test_coder_node_success(mock_state):
    valid_script = textwrap.dedent("""
        import build123d as bd
        import mujoco
        
        def build(seed: int, scale: float = 1.0):
            with bd.BuildPart() as p:
                bd.Box(10 * scale, 10 * scale, 10 * scale)
            return p.part, "<mujoco/>"
    """).strip()

    mock_response = MagicMock()
    mock_response.content = f"```python\n{valid_script}\n```"

    with patch("worker.generators.benchmark.nodes.ChatOpenAI") as mock_llm_class:
        mock_llm = mock_llm_class.return_value
        mock_llm.ainvoke = AsyncMock(return_value=mock_response)

        updated_state = await coder_node(mock_state)

        assert updated_state["current_script"] == valid_script
        assert len(updated_state["messages"]) == 1

        # Verify it's runnable
        local_scope = {}
        exec(updated_state["current_script"], local_scope)
        assert "build" in local_scope

        part, mjcf = local_scope["build"](42)
        assert part is not None
        assert mjcf == "<mujoco/>"


@pytest.mark.asyncio
async def test_coder_node_with_feedback(mock_state):
    mock_state["review_feedback"] = "Make it larger"
    mock_state["simulation_result"] = {
        "valid": False,
        "cost": 0,
        "logs": ["Intersections found"],
    }

    mock_response = MagicMock()
    mock_response.content = "```python\n# refined script\n```"

    with patch("worker.generators.benchmark.nodes.ChatOpenAI") as mock_llm_class:
        mock_llm = mock_llm_class.return_value
        mock_llm.ainvoke = AsyncMock(return_value=mock_response)

        with patch("worker.generators.benchmark.nodes.Path.read_text") as mock_read:
            mock_read.return_value = "{plan} {review_feedback} {validation_logs}"

            await coder_node(mock_state)

            # Verify the prompt contained the feedback and logs
            args, _ = mock_llm.ainvoke.call_args
            prompt_message = args[0][1].content
            assert "Make it larger" in prompt_message
            assert "Intersections found" in prompt_message


@pytest.mark.asyncio
async def test_validator_node_success(mock_state):
    from worker.generators.benchmark.nodes import validator_node

    mock_state["current_script"] = textwrap.dedent("""
        import build123d as bd
        def build(seed: int, scale: float = 1.0):
            with bd.BuildPart() as p:
                bd.Box(1, 1, 1)
            return p.part, "<mujoco/>"
    """)

    with patch("worker.utils.validation.simulate") as mock_sim:
        with patch("worker.utils.validation.validate") as mock_val:
            mock_sim.return_value.success = True
            mock_val.return_value = True

            updated_state = await validator_node(mock_state)

            assert updated_state["simulation_result"]["valid"] is True
            assert mock_sim.call_count == 2  # for seeds 0 and 42


@pytest.mark.asyncio
async def test_validator_node_failure(mock_state):
    from worker.generators.benchmark.nodes import validator_node

    mock_state["current_script"] = "invalid code"

    updated_state = await validator_node(mock_state)
    assert updated_state["simulation_result"]["valid"] is False
    assert "Validation error" in updated_state["simulation_result"]["logs"][0]
