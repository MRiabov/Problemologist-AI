import pytest
import asyncio
from unittest.mock import MagicMock, patch
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall
from src.agent.graph.graph import build_graph

@pytest.mark.asyncio
async def test_graph_simple_run():
    """
    Test a simple pass through the graph with mocked LLM.
    """
    mock_llm = MagicMock()
    mock_llm.bind_tools.return_value = mock_llm
    
    def mock_invoke(messages, **kwargs):
        # Determine which node is calling based on the system message content
        system_msg = next((m.content for m in messages if isinstance(m, SystemMessage)), "")
        
        if "Lead Mechanical Engineer (Planner)" in system_msg:
            return AIMessage(content="1. Design a cube. 2. Export it.")
        elif "CAD Engineer (Actor)" in system_msg:
            return AIMessage(content="I have started the design.")
        elif "Design Reviewer (Critic)" in system_msg:
            return AIMessage(content="Looks good.")
        return AIMessage(content="Default response")

    mock_llm.invoke.side_effect = mock_invoke
    
    with patch("src.agent.graph.nodes.planner.get_model", return_value=mock_llm), \
         patch("src.agent.graph.nodes.actor.get_model", return_value=mock_llm), \
         patch("src.agent.graph.nodes.critic.get_model", return_value=mock_llm):
        
        app = build_graph().compile()
        inputs = {"messages": [HumanMessage(content="Create a simple cube")], "plan": "", "step_count": 0}
        
        result_messages = []
        async for event in app.astream(inputs, stream_mode="updates"):
            for node_name, updates in event.items():
                if "messages" in updates:
                    msgs = updates["messages"]
                    if isinstance(msgs, list):
                        result_messages.extend(msgs)
                    else:
                        result_messages.append(msgs)
        
        assert len(result_messages) >= 2

@pytest.mark.asyncio
async def test_graph_with_tool_call():
    """
    Test the graph with a tool call and critic interaction.
    """
    mock_llm = MagicMock()
    mock_llm.bind_tools.return_value = mock_llm
    
    call_count = {"actor": 0, "planner": 0}

    def mock_invoke(messages, **kwargs):
        system_msg = next((m.content for m in messages if isinstance(m, SystemMessage)), "")
        
        if "Lead Mechanical Engineer (Planner)" in system_msg:
            call_count["planner"] += 1
            return AIMessage(content="Plan: Use preview_design")
        elif "CAD Engineer (Actor)" in system_msg:
            call_count["actor"] += 1
            if call_count["actor"] == 1:
                return AIMessage(
                    content="Previewing design",
                    tool_calls=[{
                        "name": "preview_design",
                        "args": {"path": "design.py"},
                        "id": "call_123",
                        "type": "tool_call"
                    }]
                )
            else:
                return AIMessage(content="Design finalized.")
        elif "Design Reviewer (Critic)" in system_msg:
            return AIMessage(content="The preview looks perfect. Task complete.")
        return AIMessage(content="Default response")

    mock_llm.invoke.side_effect = mock_invoke
    
    with patch("src.agent.graph.nodes.planner.get_model", return_value=mock_llm), \
         patch("src.agent.graph.nodes.actor.get_model", return_value=mock_llm), \
         patch("src.agent.graph.nodes.critic.get_model", return_value=mock_llm), \
         patch("src.agent.tools.env.preview_design_async", return_value="Preview generated: design.svg"):
        
        app = build_graph().compile()
        inputs = {"messages": [HumanMessage(content="Show me a cube")], "plan": "", "step_count": 0}
        
        nodes_visited = []
        async for event in app.astream(inputs, stream_mode="updates"):
            for node_name in event.keys():
                nodes_visited.append(node_name)
        
        assert "planner" in nodes_visited
        assert "actor" in nodes_visited
        assert "tools" in nodes_visited
        assert "critic" in nodes_visited

if __name__ == "__main__":
    asyncio.run(test_graph_simple_run())
