from unittest.mock import AsyncMock, MagicMock

import pytest

from controller.agent.initialization import initialize_agent_files
from shared.enums import AgentName


@pytest.mark.asyncio
async def test_initialize_agent_files_engineer_coder():
    # Setup
    mock_backend = MagicMock()
    mock_backend.als_info = AsyncMock(return_value=[])
    mock_backend.awrite = AsyncMock()

    # Execute
    await initialize_agent_files(mock_backend, AgentName.ENGINEER_CODER)

    # Assert
    # Verify specific files are written
    # todo.md, journal.md

    # We can't easily check the content without reading the actual files,
    # but we can check the target paths.

    # Check that awrite was called for expected files
    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "todo.md" in written_files
    assert "journal.md" in written_files
    assert "plan.md" not in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_engineer_planner():
    mock_backend = MagicMock()
    mock_backend.als_info = AsyncMock(return_value=[])
    mock_backend.awrite = AsyncMock()

    await initialize_agent_files(mock_backend, AgentName.ENGINEER_PLANNER)

    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "plan.md" in written_files
    assert "todo.md" in written_files
    assert "journal.md" in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_benchmark():
    # Setup
    mock_backend = MagicMock()
    mock_backend.als_info = AsyncMock(return_value=[])
    mock_backend.awrite = AsyncMock()

    # Execute
    await initialize_agent_files(mock_backend, AgentName.BENCHMARK_PLANNER)

    # Assert
    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "benchmark_definition.yaml" in written_files
    assert "plan.md" in written_files
    assert "todo.md" in written_files
    assert "journal.md" in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_benchmark_coder_includes_script_template():
    mock_backend = MagicMock()
    mock_backend.als_info = AsyncMock(return_value=[])
    mock_backend.awrite = AsyncMock()

    await initialize_agent_files(mock_backend, AgentName.BENCHMARK_CODER)

    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "benchmark_definition.yaml" in written_files
    assert "benchmark_assembly_definition.yaml" in written_files
    assert "script.py" in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_support():
    # Setup
    mock_backend = MagicMock()
    mock_backend.als_info = AsyncMock(return_value=[])
    mock_backend.awrite = AsyncMock()

    # Execute
    await initialize_agent_files(mock_backend, AgentName.COTS_SEARCH)

    # Assert
    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "journal.md" in written_files
    assert "plan.md" not in written_files


@pytest.mark.asyncio
async def test_initialize_agent_files_respects_existing_paths_with_leading_slash():
    mock_backend = MagicMock()
    mock_backend.als_info = AsyncMock(
        side_effect=[
            [{"path": "/benchmark_definition.yaml", "is_dir": False}],
            [],
        ]
    )
    mock_backend.awrite = AsyncMock()

    await initialize_agent_files(
        mock_backend, AgentName.BENCHMARK_PLANNER, overwrite=False
    )

    written_files = [args[0] for args, _ in mock_backend.awrite.call_args_list]
    assert "benchmark_definition.yaml" not in written_files
    assert "plan.md" in written_files
