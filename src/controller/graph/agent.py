from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
from src.controller.tools.fs import create_fs_tools


def create_agent_graph(fs_middleware: RemoteFilesystemMiddleware):
    """Create a ReAct agent graph with filesystem tools."""

    # Initialize the LLM
    # In a real scenario, we'd get the API key and model from config
    llm = ChatOpenAI(model="gpt-4o", temperature=0)

    # Create tools
    tools = create_fs_tools(fs_middleware)

    # Create the ReAct agent
    system_prompt = (
        "You are a coding agent with access to a sandboxed filesystem. "
        "Use the provided tools to explore the workspace, read/write files, "
        "and execute Python code. Always think before you act."
    )

    return create_react_agent(llm, tools, state_modifier=system_prompt)
