import pytest
from unittest.mock import AsyncMock, MagicMock
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.agent.tools import get_engineer_tools
from shared.workers.schema import ExecuteResponse

@pytest.mark.asyncio
async def test_validate_costing_and_price_tool():
    # Setup mock middleware
    fs = MagicMock(spec=RemoteFilesystemMiddleware)
    fs.validate_costing_and_price = AsyncMock(return_value=ExecuteResponse(
        success=True,
        stdout="Success: assembly_definition.yaml validated",
        stderr="",
        exit_code=0
    ))

    # Get tools
    tools = get_engineer_tools(fs, "test-session")
    validate_tool = next(t for t in tools if t.__name__ == "validate_costing_and_price")

    # Execute tool
    result = await validate_tool()

    # Assertions
    assert result["success"] is True
    assert "Success" in result["stdout"]
    fs.validate_costing_and_price.assert_called_once()

@pytest.mark.asyncio
async def test_get_docs_for_tool():
    # Setup mock middleware
    fs = MagicMock(spec=RemoteFilesystemMiddleware)
    fs.get_docs_for = AsyncMock(return_value="### Documentation for Box")

    # Get tools
    tools = get_engineer_tools(fs, "test-session")
    docs_tool = next(t for t in tools if t.__name__ == "get_docs_for")

    # Execute tool
    result = await docs_tool("Box")

    # Assertions
    assert "Documentation for Box" in result
    fs.get_docs_for.assert_called_once_with("Box")
