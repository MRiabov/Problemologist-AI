import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.models import GenerationSession
from controller.agent.benchmark.nodes import (
    summarizer_node as benchmark_summarizer_node,
)
from controller.agent.benchmark.state import BenchmarkGeneratorState
from controller.agent.nodes.summarizer import summarizer_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock(return_value="# Mock content")
        instance.exists = AsyncMock(return_value=True)
        yield instance


@pytest.mark.asyncio
async def test_summarizer_node_skips_short_journal(mock_worker):
    state = AgentState(journal="Short journal", session_id="test-session")
    result = await summarizer_node(state)
    assert result.journal == "Short journal"


@pytest.mark.asyncio
async def test_summarizer_node_compresses_long_journal(mock_worker):
    long_journal = "X" * 6000
    state = AgentState(journal=long_journal, session_id="test-session")

    with patch("dspy.ReAct") as mock_predict:
        instance = mock_predict.return_value
        instance.return_value = MagicMock(summarized_journal="Summarized content")

        result = await summarizer_node(state)

    assert "Summarized Journal" in result.journal
    assert "Summarized content" in result.journal


@pytest.mark.asyncio
async def test_benchmark_summarizer_node_compresses_long_journal(mock_worker):
    long_journal = "X" * 6000
    session_id = uuid.uuid4()
    session = GenerationSession(session_id=session_id, prompt="test")
    state = BenchmarkGeneratorState(session=session, journal=long_journal)

    with patch("dspy.ReAct") as mock_predict:
        instance = mock_predict.return_value
        instance.return_value = MagicMock(summarized_journal="Benchmark summary")

        result = await benchmark_summarizer_node(state)

    assert "Summarized Journal" in result.journal
    assert "Benchmark summary" in result.journal
