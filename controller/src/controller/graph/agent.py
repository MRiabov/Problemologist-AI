from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.tools.fs import create_fs_tools
from controller.observability.langfuse import get_langfuse_callback


def create_agent_graph(fs_middleware: RemoteFilesystemMiddleware):
    """Create a ReAct agent graph with filesystem tools."""

    # Initialize the LLM
    # In a real scenario, we'd get the API key and model from config
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    
    # Try to get Langfuse callback
    langfuse_callback = get_langfuse_callback()
    callbacks = [langfuse_callback] if langfuse_callback else []

    # Create tools
    tools = create_fs_tools(fs_middleware)

    # Create the ReAct agent
    system_prompt = (
        "You are a coding agent with access to a sandboxed filesystem. "
        "Use the provided tools to explore the workspace, read/write files, "
        "and execute Python code. Always think before you act."
    )

    # We return the agent. Callers should pass callbacks to invoke if they want tracing.
    # However, create_react_agent doesn't store callbacks.
    # We'll modify the invoke call in main.py if needed, 
    # but some people prefer binding callbacks to the LLM.
    if callbacks:
        llm = llm.with_config({"callbacks": callbacks})

    return create_react_agent(llm, tools, state_modifier=system_prompt)
