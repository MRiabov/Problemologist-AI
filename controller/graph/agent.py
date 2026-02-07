from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent

from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.langfuse import get_langfuse_callback
from controller.tools.fs import create_fs_tools
from controller.prompts import get_prompt
from shared.logging import get_logger

logger = get_logger(__name__)


def create_agent_graph(fs_middleware: RemoteFilesystemMiddleware):
    """Create a ReAct agent graph with filesystem tools."""

    # Initialize the LLM
    import os

    model = os.getenv("LLM_MODEL", "gpt-4o")
    base_url = os.getenv("OPENAI_API_BASE")
    temperature = float(os.getenv("LLM_TEMPERATURE", "0.0"))

    llm = ChatOpenAI(
        model=model,
        temperature=temperature,
        base_url=base_url,
    )

    # Try to get Langfuse callback
    langfuse_callback = get_langfuse_callback()
    callbacks = [langfuse_callback] if langfuse_callback else []

    # Create tools
    tools = create_fs_tools(fs_middleware)

    # Load system prompt from config
    try:
        system_prompt = get_prompt("cad_agent.planner.system")
    except Exception as e:
        logger.warning("failed_to_load_custom_prompt", error=str(e))
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

    return create_react_agent(llm, tools, prompt=system_prompt)
