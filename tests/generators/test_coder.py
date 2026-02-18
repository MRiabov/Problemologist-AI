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
        plan={"theme": "bracket", "reasoning": "A simple L-bracket"},
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

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.create"
        ) as mock_ctx_create,
        patch("controller.agent.benchmark.nodes.dspy.CodeAct") as mock_codeact,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.read_file.return_value = valid_script
        mock_ctx.worker_client.list_files.return_value = []
        mock_ctx.get_callbacks.return_value = []

        mock_program = MagicMock()
        mock_program.return_value = MagicMock(journal="Done")
        mock_codeact.return_value = mock_program

        updated_state = await coder_node(mock_state)

    assert updated_state.current_script == valid_script
    # HumanMessage + HumanMessage (from result)
    assert len(updated_state.messages) == 1

    # Verify it's runnable
    local_scope = {}
    exec(updated_state.current_script, local_scope)
    assert "build" in local_scope

    part, mjcf = local_scope["build"](42)
    assert part is not None
    assert mjcf == "<mujoco/>"


@pytest.mark.asyncio
async def test_coder_node_with_feedback(mock_state):
    mock_state.review_feedback = "Make it larger"
    from shared.simulation.schemas import ValidationResult

    mock_state.simulation_result = ValidationResult(
        valid=False,
        cost=0,
        logs=["Intersections found"],
    )

    # Patch dependencies
    with (
        patch(
            "controller.agent.benchmark.nodes.SharedNodeContext.create"
        ) as mock_ctx_create,
        patch("controller.agent.benchmark.nodes.dspy.CodeAct") as mock_codeact,
        patch("controller.agent.benchmark.nodes.get_benchmark_tools") as mock_get_tools,
    ):
        mock_ctx = MagicMock()
        mock_ctx_create.return_value = mock_ctx
        mock_ctx.worker_client = AsyncMock()
        mock_ctx.worker_client.read_file.return_value = "# refined script"
        mock_ctx.worker_client.list_files.return_value = []
        mock_ctx.get_callbacks.return_value = []

        mock_program = MagicMock()
        mock_program.return_value = MagicMock(journal="Done")
        mock_codeact.return_value = mock_program

        await coder_node(mock_state)

    # Verify agent was called
    mock_program.assert_called_once()
    call_args = mock_program.call_args
    kwargs = call_args.kwargs
    assert "Make it larger" in kwargs["review_feedback"]
    assert "Intersections found" in kwargs["validation_logs"]
