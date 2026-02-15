from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.tools.fs import create_fs_tools


@pytest.fixture(autouse=True)
def mock_observability():
    with (
        patch(
            "controller.middleware.remote_fs.record_worker_events",
            new_callable=AsyncMock,
        ) as m1,
        patch(
            "controller.middleware.remote_fs.sync_asset", new_callable=AsyncMock
        ) as m2,
        patch(
            "controller.observability.broadcast.EpisodeBroadcaster",
            new_callable=MagicMock,
        ) as m3,
    ):
        # Mock EpisodeBroadcaster.get_instance().broadcast
        m3.get_instance.return_value.broadcast = AsyncMock()
        yield m1, m2, m3


@pytest.mark.asyncio
async def test_remote_fs_middleware_ls():
    mock_client = AsyncMock()
    mock_client.list_files.return_value = [{"name": "test.txt", "type": "file"}]

    middleware = RemoteFilesystemMiddleware(mock_client)
    result = await middleware.list_files("/")

    assert result == [{"name": "test.txt", "type": "file"}]
    mock_client.list_files.assert_called_once_with("/")


@pytest.mark.asyncio
async def test_remote_fs_middleware_write_protection():
    mock_client = AsyncMock()
    middleware = RemoteFilesystemMiddleware(mock_client)

    # Writing to a protected path should raise PermissionError
    with pytest.raises(PermissionError, match="read-only"):
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
    result = await middleware.read_file("test.py")


@pytest.mark.asyncio
async def test_remote_fs_middleware_overwrite(mock_observability):
    m1, m2, m3 = mock_observability
    mock_client = AsyncMock()
    middleware = RemoteFilesystemMiddleware(mock_client)

    # Test with overwrite=True
    await middleware.write_file("test.txt", "content", overwrite=True)
    mock_client.write_file.assert_called_with("test.txt", "content", overwrite=True)

    # Check event recording
    call_args = m1.call_args_list[0]
    event = call_args.kwargs["events"][0]
    assert event.overwrite is True

    # Test with overwrite=False
    mock_client.write_file.reset_mock()
    m1.reset_mock()
    await middleware.write_file("test.txt", "content", overwrite=False)
    mock_client.write_file.assert_called_with("test.txt", "content", overwrite=False)

    call_args = m1.call_args_list[0]
    event = call_args.kwargs["events"][0]
    assert event.overwrite is False
