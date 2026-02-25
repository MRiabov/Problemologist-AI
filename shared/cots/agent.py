import os

import dspy

from .models import SearchConstraints, SearchQuery
from .runtime import search_parts

# Default DB path relative to project root or configurable
DEFAULT_DB_PATH = os.environ.get("COTS_DB_PATH", "parts.db")


def search_cots_catalog(
    query: str,
    max_weight_g: float | None = None,
    max_cost: float | None = None,
    category: str | None = None,
    limit: int = 5,
) -> str:
    """
    Search for off-the-shelf mechanical and electronic components (fasteners, bearings, etc.).
    Returns a list of parts with their IDs, costs, weights, and Python import recipes.

    Args:
        query: Text description of the part (e.g. 'M6 hex nut').
        max_weight_g: Maximum allowed weight in grams.
        max_cost: Maximum allowed unit cost.
        category: Filter by category ('fastener', 'motor', 'gear', 'bearing', 'electronic').
        limit: Max number of results.
    """
    constraints = SearchConstraints(
        max_weight_g=max_weight_g,
        max_cost=max_cost,
        category=category,
    )

    sq = SearchQuery(query=query, constraints=constraints, limit=limit)
    parts = search_parts(sq, DEFAULT_DB_PATH)

    if not parts:
        return "No parts found matching the criteria."

    output = []
    for p in parts:
        item_str = (
            f"- {p.name} (ID: {p.part_id})\n"
            f"  Category: {p.category}, Weight: {p.weight_g:.2f}g, Cost: ${p.unit_cost}\n"
            f"  Recipe: {p.import_recipe}"
        )
        output.append(item_str)

    return "\n\n".join(output)


class COTSSearchSignature(dspy.Signature):
    """
    You are a COTS (Commercial Off-The-Shelf) assembly assistant.
    Your goal is to find the best components for a given design requirement.
    Use the search_cots_catalog tool to find parts.
    If you find multiple candidates, recommend the best fit based on constraints.
    """

    requirement = dspy.InputField(desc="User design requirement")
    recommendation = dspy.OutputField(desc="Recommended parts with reasoning")


class DSPyLangGraphWrapper:
    """Wrapper to make a DSPy module look like a LangGraph for ainvoke."""

    def __init__(self, program: dspy.Module):
        self.program = program

    async def ainvoke(self, input_data: dict, _config: dict | None = None) -> dict:
        """Compatibility layer for execute_agent_task."""
        # Extract the task/message from input_data
        messages = input_data.get("messages", [])
        if messages:
            task = messages[-1].content
        else:
            task = input_data.get("task", "Search for a COTS part.")

        # Run the DSPy program. COTSSearchSignature uses 'requirement'.
        result = self.program(requirement=task)

        # Return in a format expected by tasks.py
        from langchain_core.messages import AIMessage

        # Access recommendation field safely
        content = (
            result.recommendation if hasattr(result, "recommendation") else str(result)
        )
        return {"messages": [AIMessage(content=content)]}


def create_cots_search_agent(_model_name: str):
    """
    Create a specialized agent for part lookup using DSPy.
    Returns a wrapped DSPy module for LangGraph compatibility.
    """
    program = dspy.ReAct(COTSSearchSignature, tools=[search_cots_catalog])
    return DSPyLangGraphWrapper(program)
