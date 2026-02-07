import asyncio
import uuid
from unittest.mock import MagicMock

import pytest
from fastapi import HTTPException, Response

from controller.observability.broadcast import EpisodeBroadcaster
from worker.api.routes import get_asset


@pytest.mark.asyncio
async def test_episode_broadcaster():
    broadcaster = EpisodeBroadcaster.get_instance()
    episode_id = uuid.uuid4()

    # Create a subscriber
    async def subscriber():
        messages = []
        async for msg in broadcaster.subscribe(episode_id):
            messages.append(msg)
            if len(messages) >= 2:
                break
        return messages

    # Start subscriber task
    sub_task = asyncio.create_task(subscriber())

    # Allow subscriber to start
    await asyncio.sleep(0.1)

    # Broadcast messages
    await broadcaster.broadcast(episode_id, {"data": "msg1"})
    await broadcaster.broadcast(episode_id, {"data": "msg2"})

    # Wait for subscriber
    messages = await sub_task
    assert len(messages) == 2
    assert messages[0] == {"data": "msg1"}
    assert messages[1] == {"data": "msg2"}


@pytest.mark.asyncio
async def test_get_asset_glb():
    # Mock fs_router
    mock_fs_router = MagicMock()
    mock_fs_router.read.return_value = b"glb_content"
    # exists returns False to bypass source check for this test
    mock_fs_router.exists.return_value = False

    response = await get_asset("model.glb", fs_router=mock_fs_router)

    assert isinstance(response, Response)
    assert response.body == b"glb_content"
    assert response.media_type == "model/gltf-binary"
    mock_fs_router.read.assert_called_with("model.glb")


@pytest.mark.asyncio
async def test_get_asset_py():
    # Mock fs_router
    mock_fs_router = MagicMock()
    mock_fs_router.read.return_value = b"print('hello')"

    response = await get_asset("script.py", fs_router=mock_fs_router)

    assert response.body == b"print('hello')"
    assert response.media_type == "text/x-python"


@pytest.mark.asyncio
async def test_get_asset_syntax_error():
    # Mock fs_router
    mock_fs_router = MagicMock()

    # Simulate a python file existing and having syntax error
    mock_fs_router.exists.side_effect = lambda path: path == "model.py"
    mock_fs_router.read.side_effect = (
        lambda path: b"def broken_code(" if path == "model.py" else b"ignored"
    )

    # Expect HTTPException 422
    with pytest.raises(HTTPException) as excinfo:
        await get_asset("model.glb", fs_router=mock_fs_router)

    assert excinfo.value.status_code == 422
    assert "syntax errors" in excinfo.value.detail
