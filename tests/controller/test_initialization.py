import pytest
from unittest.mock import MagicMock, AsyncMock

from controller.agent.initialization import initialize_agent_files


@pytest.mark.asyncio
async def test_initialize_agent_files_engineer():
    # Setup
    mock_backend = MagicMock()
    mock_backend.awrite = AsyncMock()

    # Execute
    await initialize_agent_files(mock_backend, "engineer_coder")

    # Assert
    # Verify specific files are written
    # plan.md, todo.md, journal.md

    # We can't easily check the content without reading the actual files,
    # but we can check the target paths.

    # Check that awrite was called for expected files
    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "plan.md" in written_files
    assert "todo.md" in written_files
    assert "journal.md" in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_benchmark():
    # Setup
    mock_backend = MagicMock()
    mock_backend.awrite = AsyncMock()

    # Execute
    await initialize_agent_files(mock_backend, "benchmark_planner")

    # Assert
    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "objectives.yaml" in written_files
    assert "plan.md" in written_files
    assert "todo.md" in written_files
    assert "journal.md" in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_support():
    # Setup
    mock_backend = MagicMock()
    mock_backend.awrite = AsyncMock()

    # Execute
    await initialize_agent_files(mock_backend, "cots_search")

    # Assert
    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "journal.md" in written_files
    assert "plan.md" not in written_files
