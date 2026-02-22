import pytest
import uuid
from unittest.mock import MagicMock, AsyncMock, patch
from langchain_core.messages import AIMessage
from controller.agent.benchmark.nodes import BenchmarkSkillsNode, SharedNodeContext, BenchmarkGeneratorState
from controller.agent.benchmark.models import GenerationSession
from shared.enums import SessionStatus

@pytest.mark.asyncio
async def test_benchmark_skills_node():
    # Mock dependencies
    mock_ctx = MagicMock(spec=SharedNodeContext)
    mock_ctx.worker_client = AsyncMock()
    mock_ctx.fs = AsyncMock()
    mock_ctx.dspy_lm = MagicMock()
    mock_ctx.session_id = str(uuid.uuid4())
    mock_ctx.pm = MagicMock()
    mock_ctx.pm.render.return_value = "system prompt"

    # Mock state
    session = GenerationSession(
        session_id=uuid.UUID(mock_ctx.session_id),
        prompt="Create a benchmark",
        status=SessionStatus.ACCEPTED,
        validation_logs=["Log 1", "Log 2"]
    )
    state = BenchmarkGeneratorState(
        session=session,
        messages=[AIMessage(content="Message 1"), AIMessage(content="Message 2")]
    )

    # Mock GitManager
    with patch("controller.agent.benchmark.nodes.GitManager") as MockGitManager:
        mock_git = MockGitManager.return_value
        mock_git.ensure_repo = MagicMock()
        mock_git.sync_changes = AsyncMock()

        node = BenchmarkSkillsNode(context=mock_ctx)

        # Mock _run_program to avoid actual DSPy execution
        mock_prediction = MagicMock()
        mock_prediction.summary = "Identified skill: optimized_loop"
        node._run_program = AsyncMock(return_value=(mock_prediction, {}, "Journal Entry"))

        new_state = await node(state)

        # Verify
        assert "Skills update: Identified skill: optimized_loop" in new_state.messages[-1].content
        node._run_program.assert_called_once()

        # Check inputs passed to _run_program
        call_args = node._run_program.call_args
        assert call_args
        # _run_program signature: (self, program_cls, signature_cls, state, inputs, tool_factory, validate_files, node_type, max_retries=3)
        # args[0] is program_cls, args[1] is signature_cls, args[2] is state, args[3] is inputs

        if len(call_args.args) >= 4:
             inputs = call_args.args[3]
        else:
             inputs = call_args.kwargs.get("inputs")

        assert inputs["task"] == "Create a benchmark"
        assert "Message 1" in inputs["journal"]
        assert "Log 1" in inputs["journal"]
