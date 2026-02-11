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

    with patch("controller.agent.benchmark.nodes.ChatOpenAI"):
        with patch(
            "controller.agent.benchmark.nodes.create_deep_agent"
        ) as mock_create_agent:
            mock_agent = mock_create_agent.return_value
            mock_agent.ainvoke = AsyncMock(return_value={"messages": [mock_response]})

            with patch(
                "controller.agent.benchmark.nodes.WorkerClient"
            ) as mock_client_class:
                mock_client = mock_client_class.return_value
                mock_client.read_file = AsyncMock(return_value=valid_script)
                mock_client.list_files = AsyncMock(return_value=[])

                with patch("controller.agent.benchmark.nodes.RemoteFilesystemBackend"):
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

    with patch("controller.agent.benchmark.nodes.ChatOpenAI"):
        with patch(
            "controller.agent.benchmark.nodes.create_deep_agent"
        ) as mock_create_agent:
            mock_agent = mock_create_agent.return_value
            mock_agent.ainvoke = AsyncMock(return_value={"messages": [mock_response]})

            with patch(
                "controller.agent.benchmark.nodes.WorkerClient"
            ) as mock_client_class:
                mock_client = mock_client_class.return_value
                mock_client.read_file = AsyncMock(return_value="# refined script")
                mock_client.list_files = AsyncMock(return_value=[])

                with patch("controller.agent.benchmark.nodes.RemoteFilesystemBackend"):
                    await coder_node(mock_state)

            # Verify create_deep_agent was called with correct system prompt
            args, kwargs = mock_create_agent.call_args
            system_prompt = kwargs.get("system_prompt", "")
            assert "Make it larger" in system_prompt
            assert "Intersections found" in system_prompt


@pytest.mark.asyncio
async def test_validator_node_success(mock_state):
    from controller.agent.benchmark.nodes import validator_node

    mock_state["current_script"] = textwrap.dedent("""
        import build123d as bd
        def build(seed: int, scale: float = 1.0):
            with bd.BuildPart() as p:
                bd.Box(1, 1, 1)
            return p.part, "<mujoco/>"
    """)

    with patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_class:
        mock_client = mock_client_class.return_value
        mock_client.git_commit = AsyncMock(
            return_value=MagicMock(success=True, commit_hash="hash")
        )
        mock_client.validate = AsyncMock(return_value=MagicMock(success=True))
        mock_client.simulate = AsyncMock(
            return_value=MagicMock(success=True, artifacts={"render_paths": []})
        )

        updated_state = await validator_node(mock_state)

        assert updated_state["simulation_result"]["valid"] is True
        assert mock_client.simulate.call_count == 1


@pytest.mark.asyncio
async def test_validator_node_failure(mock_state):
    from controller.agent.benchmark.nodes import validator_node

    mock_state["current_script"] = "invalid code"

    with patch("controller.agent.benchmark.nodes.WorkerClient") as mock_client_class:
        mock_client = mock_client_class.return_value
        mock_client.git_commit = AsyncMock(return_value=MagicMock(success=True))
        mock_client.validate = AsyncMock(
            return_value=MagicMock(success=False, message="Validation error")
        )

        updated_state = await validator_node(mock_state)
        assert updated_state["simulation_result"]["valid"] is False
        assert "Validation error" in updated_state["simulation_result"]["logs"][0]
