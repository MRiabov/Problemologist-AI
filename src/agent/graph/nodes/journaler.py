from langchain_core.messages import SystemMessage, ToolMessage

from src.agent.graph.state import AgentState
from src.agent.tools.memory import write_journal
from src.agent.utils.config import Config
from src.agent.utils.llm import get_model

JOURNALER_PROMPT = """You are the Knowledge Manager (Journaler).
The current design task has been successfully completed.
Your goal is to reflect on the session and record any key lessons,
successful patterns, or reusable code snippets into the long-term memory (Journal).

Review the conversation history and identify:
1. What was the problem?
2. What was the winning solution (code snippet or strategy)?
3. Any tricky syntax or documentation gaps that we solved?

Call the 'write_journal' tool to save this information.
"""


def journaler_node(state: AgentState):
    """
    Reflects on the session and updates the journal.
    """
    model = get_model(Config.LLM_MODEL)

    # Bind tools to the model
    tools = [write_journal]
    model_with_tools = model.bind_tools(tools)

    messages = [SystemMessage(content=JOURNALER_PROMPT)] + state["messages"]

    response = model_with_tools.invoke(messages)

    # Check for tool calls and execute them immediately
    # This avoids routing complexity in the main graph for this terminal node
    tool_calls = getattr(response, "tool_calls", [])
    results = []
    if tool_calls:
        for tc in tool_calls:
            if tc["name"] == "write_journal":
                try:
                    # write_journal is a structured tool, invoke with args dict
                    output = write_journal.invoke(tc["args"])
                    results.append(
                        ToolMessage(
                            content=str(output),
                            tool_call_id=tc["id"],
                            name=tc["name"],
                        )
                    )
                except Exception as e:
                    results.append(
                        ToolMessage(
                            content=f"Error: {e}",
                            tool_call_id=tc["id"],
                            name=tc["name"],
                        )
                    )

    return {"messages": [response, *results]}
