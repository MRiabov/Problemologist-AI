import textwrap
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import coder_node, extract_python_code
from controller.agent.benchmark.state import BenchmarkGeneratorState


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


@patch("controller.agent.benchmark.nodes.ChatOpenAI")
@pytest.mark.asyncio
async def test_coder_node_success(mock_llm_cls, mock_state):
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
    mock_response.tool_calls = []

    # Mock LLM
    mock_llm = mock_llm_cls.return_value
    mock_llm.ainvoke = AsyncMock(return_value=mock_response)
    mock_llm.bind_tools = MagicMock(return_value=mock_llm)

    with patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_class:
        mock_client = mock_client_class.return_value
        mock_client.read_file = AsyncMock(return_value=valid_script)
        mock_client.list_files = AsyncMock(return_value=[])

        with patch("controller.agent.benchmark.nodes.RemoteFilesystemMiddleware"):
            updated_state = await coder_node(mock_state)

    assert updated_state["current_script"] == valid_script
    # HumanMessage + 1st response (loop breaks)
    assert len(updated_state["messages"]) == 2

    # Verify it's runnable
    local_scope = {}
    exec(updated_state["current_script"], local_scope)
    assert "build" in local_scope

    part, mjcf = local_scope["build"](42)
    assert part is not None
    assert mjcf == "<mujoco/>"


@patch("controller.agent.benchmark.nodes.ChatOpenAI")
@pytest.mark.asyncio
async def test_coder_node_with_feedback(mock_llm_cls, mock_state):
    mock_state["review_feedback"] = "Make it larger"
    mock_state["simulation_result"] = {
        "valid": False,
        "cost": 0,
        "logs": ["Intersections found"],
    }

    mock_response = MagicMock()
    mock_response.content = "```python\n# refined script\n```"
    mock_response.tool_calls = []

    # Mock LLM
    mock_llm = mock_llm_cls.return_value
    mock_llm.ainvoke = AsyncMock(return_value=mock_response)
    mock_llm.bind_tools = MagicMock(return_value=mock_llm)

    with patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_class:
        mock_client = mock_client_class.return_value
        mock_client.read_file = AsyncMock(return_value="# refined script")
        mock_client.list_files = AsyncMock(return_value=[])

        with patch("controller.agent.benchmark.nodes.RemoteFilesystemMiddleware"):
            await coder_node(mock_state)

    # Verify LLM was called with correct context
    call_args = mock_llm.ainvoke.call_args
    messages = call_args.args[0]
    system_prompt = messages[0].content
    assert "Make it larger" in system_prompt
    assert "Intersections found" in system_prompt
