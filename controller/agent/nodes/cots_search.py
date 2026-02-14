import logging
from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI
from shared.cots.agent import search_cots_catalog
from shared.type_checking import type_check
from ..config import settings
from ..state import AgentState

logger = logging.getLogger(__name__)

@type_check
async def cots_search_node(state: AgentState) -> AgentState:
    """
    COTS Search node: Searches for components based on current needs.
    """
    llm = ChatOpenAI(model=settings.llm_model, temperature=0)

    # We want the LLM to decide what to search for based on the task and plan
    prompt = f"""You are a COTS search assistant.
Based on the following task and current plan, determine what components (motors, fasteners, bearings, etc.) need to be searched for in the catalog.
If you have enough information, perform the searches using the tools.

Task: {state.task}
Plan: {state.plan}
Journal: {state.journal}

Identify specific search queries.
"""

    # For now, let's keep it simple and just use the search tool if requested in the journal or plan
    # Actually, a better way is to let the LLM use the tool.

    from langchain_core.tools import tool
    from langgraph.prebuilt import create_react_agent

    tools = [search_cots_catalog]
    agent = create_react_agent(llm, tools)

    result = await agent.ainvoke({
        "messages": [HumanMessage(content=prompt)]
    })

    new_journal = state.journal + "\n[COTS Search] Completed search for components."
    # We could add more detail from the result if needed

    return state.model_copy(update={"journal": new_journal, "messages": state.messages + result["messages"]})
