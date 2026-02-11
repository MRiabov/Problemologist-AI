from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from dotenv import load_dotenv

from controller.clients.backend import RemoteFilesystemBackend
from controller.graph.agent import create_agent_graph

# Load environment variables from .env
load_dotenv()


@pytest.mark.integration
@pytest.mark.asyncio
async def test_agent_run_real_llm():
    """
    Integration test that queries an actual LLM (GLM 4.7-flash).
    Verifies that create_agent_graph correctly initializes the agent and
    that it can be invoked with a real model.
    """

    # We rely on OPENAI_API_KEY and OPENAI_API_BASE being set in the environment.
    # We patch get_prompt to provide a simple test prompt.
    with patch("controller.graph.agent.get_prompt") as mock_get_prompt:
        mock_get_prompt.return_value = "This is a test. Your task is to ONLY return the word 'SUCCESS' and nothing else."

        # We also need a mock backend because we don't want to hit a real worker in this test.
        # But we want to test the LLM part.
        mock_backend = MagicMock(spec=RemoteFilesystemBackend)
        mock_backend.als_info = AsyncMock(return_value=[])
        mock_backend.aread = AsyncMock(return_value="")
        mock_backend.awrite = AsyncMock()
        mock_backend.aedit = AsyncMock()

        # Create agent
        # settings will be loaded naturally from .env
        agent, _ = create_agent_graph(mock_backend)

        # Invoke agent
        # The prompt is in the state_modifier, so we just send a trigger message.
        response = await agent.ainvoke({"messages": [("user", "Run the test.")]})

        # Verify response
        last_message = response["messages"][-1].content
        print(f"Agent response: {last_message}")

        assert "SUCCESS" in last_message.upper()
