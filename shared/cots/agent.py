import os
from typing import Any

import dspy

from .models import SearchQuery
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
    constraints = {}
    if max_weight_g is not None:
        constraints["max_weight_g"] = max_weight_g
    if max_cost is not None:
        constraints["max_cost"] = max_cost
    if category is not None:
        constraints["category"] = category

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
    If you find multiple candidates, recommend the best fit based on the user's constraints (weight, cost, size).
    """

    requirement = dspy.InputField(desc="User design requirement")
    recommendation = dspy.OutputField(desc="Recommended parts with reasoning")


def create_cots_search_agent(model_name: str):
    """
    Create a specialized agent for part lookup using DSPy.
    Note: Returns a DSPy module, which might need wrapping if used in a LangGraph.
    """
    # WP11: Migrated from LangGraph ReAct to DSPy CodeAct
    # Since it was used in controller/graph/agent.py which expects a LangGraph,
    # we might need to wrap it if we want full backward compatibility.
    # However, the user wants native DSPy integration.

    # For now, let's just make it a simple CodeAct module.
    program = dspy.CodeAct(COTSSearchSignature, tools=[search_cots_catalog])
    return program
