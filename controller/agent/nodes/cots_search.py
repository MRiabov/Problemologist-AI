import logging
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent

from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.type_checking import type_check
from ..config import settings
from ..state import AgentState
from ..tools import get_engineer_tools

logger = logging.getLogger(__name__)


@type_check
async def cots_search_node(state: AgentState) -> AgentState:
    """
    COTS Search node: Searches for components based on current needs.
    Refactored to use create_react_agent and standard tools.
    """
    session_id = state.session_id or settings.default_session_id
    worker_url = settings.spec_001_api_url

    # Initialize workspace client and tools
    worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
    fs = RemoteFilesystemMiddleware(worker_client)
    tools = get_engineer_tools(fs, session_id)

    llm = ChatOpenAI(model=settings.llm_model, temperature=0)
    agent = create_react_agent(llm, tools)

    prompt = f"""You are a COTS search assistant.
Based on the following task and current plan, determine what components (motors, fasteners, bearings, etc.) need to be searched for in the catalog.
If you have enough information, perform the searches using the tools.

Task: {state.task}
Plan: {state.plan}
Journal: {state.journal}

Identify specific search queries.
"""

    messages = [SystemMessage(content=prompt)]

    try:
        result = await agent.ainvoke({"messages": messages})
        final_messages = result["messages"]

        new_journal = state.journal + "\n[COTS Search] Completed search for components."

        return state.model_copy(
            update={"journal": new_journal, "messages": state.messages + final_messages}
        )
    except Exception as e:
        logger.error("COTS Search agent failed", error=str(e))
        return state.model_copy(
            update={"journal": state.journal + f"\n[COTS Search] System error: {e}"}
        )
