from unittest.mock import AsyncMock, patch

import pytest

from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.tools.fs import create_fs_tools
from shared.workers.schema import GrepMatch


@pytest.fixture(autouse=True)
def mock_observability():
    with (
        patch(
            "controller.middleware.remote_fs.record_events",
            new_callable=AsyncMock,
        ) as m1,
        patch(
            "controller.middleware.remote_fs.broadcast_file_update",
            new_callable=AsyncMock,
        ) as m2,
    ):
        yield m1, m2


@pytest.mark.asyncio
async def test_remote_fs_middleware_ls():
    mock_client = AsyncMock()
    mock_client.list_files.return_value = [{"name": "test.txt", "type": "file"}]

    middleware = RemoteFilesystemMiddleware(mock_client)
    result = await middleware.list_files("/")

    assert result == [{"name": "test.txt", "type": "file"}]
    mock_client.list_files.assert_called_once_with("/")


@pytest.mark.asyncio
async def test_remote_fs_middleware_ls_filters_unreadable_entries():
    mock_client = AsyncMock()
    mock_client.list_files.return_value = [
        {"path": "/plan.md", "name": "plan.md", "is_dir": False, "size": 10},
        {
            "path": "/unmatched.txt",
            "name": "unmatched.txt",
            "is_dir": False,
            "size": 10,
        },
    ]

    middleware = RemoteFilesystemMiddleware(mock_client)
    result = await middleware.list_files("/")

    assert len(result) == 1
    assert result[0]["path"] == "/plan.md"


@pytest.mark.asyncio
async def test_remote_fs_middleware_write_protection():
    mock_client = AsyncMock()
    middleware = RemoteFilesystemMiddleware(mock_client)

    # Writing to a protected path should raise PermissionError
    with pytest.raises(PermissionError, match="Permission denied"):
        await middleware.write_file("skills/test.py", "print(1)")

    mock_client.write_file.assert_not_called()


@pytest.mark.asyncio
async def test_fs_tools_execution():
    mock_middleware = AsyncMock()
    mock_middleware.run_command.return_value = {
        "stdout": "hello",
        "stderr": "",
        "exit_code": 0,
    }

    tools = create_fs_tools(mock_middleware)
    exec_tool = next(t for t in tools if t.name == "execute_python")

    result = await exec_tool.ainvoke({"code": "print('hello')", "timeout": 10})

    assert result == {"stdout": "hello", "stderr": "", "exit_code": 0}
    mock_middleware.run_command.assert_called_once_with(
        code="print('hello')", timeout=10
    )


@pytest.mark.asyncio
async def test_remote_fs_middleware_read():
    mock_client = AsyncMock()
    mock_client.read_file.return_value = "file content"

    middleware = RemoteFilesystemMiddleware(mock_client)
    result = await middleware.read_file("plan.md")
    assert result == "file content"


@pytest.mark.asyncio
async def test_remote_fs_middleware_overwrite(mock_observability):
    m1, m2 = mock_observability
    mock_client = AsyncMock()
    middleware = RemoteFilesystemMiddleware(mock_client)

    # Test with overwrite=True
    await middleware.write_file("todo.md", "content", overwrite=True)
    mock_client.write_file.assert_called_with("todo.md", "content", overwrite=True)

    # Check event recording
    call_args = m1.call_args_list[0]
    event = call_args.kwargs["events"][0]
    assert event.overwrite is True

    # Test with overwrite=False
    mock_client.write_file.reset_mock()
    m1.reset_mock()
    await middleware.write_file("todo.md", "content", overwrite=False)
    mock_client.write_file.assert_called_with("todo.md", "content", overwrite=False)

    call_args = m1.call_args_list[0]
    event = call_args.kwargs["events"][0]
    assert event.overwrite is False


@pytest.mark.asyncio
async def test_remote_fs_middleware_grep_filters_unreadable_matches():
    mock_client = AsyncMock()
    mock_client.grep.return_value = [
        GrepMatch(path="/plan.md", line=1, text="ok"),
        GrepMatch(path="/script.py", line=2, text="blocked"),
    ]
    middleware = RemoteFilesystemMiddleware(mock_client)

    # Mock default policy which allows all for engineer role
    # But wait, RemoteFilesystemMiddleware uses get_fs_policy() by default.
    # The default agent_role is 'engineering_mechanical_coder'.
    # In config/agents_config.yaml, 'engineering_mechanical_coder' can read everything.
    # We need to use a role that has some denials to test filtering.

    middleware.agent_role = "engineering_planner"
    # engineering_planner denies script.py and **/*.py for writing, but for reading?
    # Actually, the config says engineering_planner allows config/**, skills/**, utils/**,
    # objectives.yaml, assembly_definition.yaml, plan.md, todo.md, journal.md.
    # It does NOT allow script.py.

    result = await middleware.grep("pattern", path="/")

    assert len(result) == 1
    assert result[0].path == "/plan.md"


@pytest.mark.asyncio
async def test_remote_fs_middleware_analyze():
    mock_client = AsyncMock()
    from shared.workers.workbench_models import WorkbenchResult, WorkbenchMetadata, CostBreakdown
    from shared.enums import ManufacturingMethod

    mock_result = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.0,
        weight_g=100.0,
        metadata=WorkbenchMetadata(
            cost_breakdown=CostBreakdown(
                process="cnc",
                total_cost=10.0,
                unit_cost=10.0,
                material_cost_per_unit=2.0,
                setup_cost=8.0,
                is_reused=False
            )
        )
    )
    mock_client.analyze.return_value = mock_result

    middleware = RemoteFilesystemMiddleware(mock_client)
    result = await middleware.analyze("script.py", method=ManufacturingMethod.CNC, quantity=1)

    assert result.is_manufacturable is True
    assert result.unit_cost == 10.0
    assert result.weight_g == 100.0
    mock_client.analyze.assert_called_once_with(
        method=ManufacturingMethod.CNC,
        script_path="script.py",
        quantity=1
    )
