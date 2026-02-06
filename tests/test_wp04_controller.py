import pytest
from unittest.mock import AsyncMock
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
from src.controller.tools.fs import create_fs_tools


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

    assert result == "file content"
    mock_client.read_file.assert_called_once_with("test.py")
