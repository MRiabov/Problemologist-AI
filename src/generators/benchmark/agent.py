import traceback
from pathlib import Path
from typing import Annotated, Literal, TypedDict

import yaml
from langchain_core.messages import AnyMessage, HumanMessage, SystemMessage
from langgraph.graph import END, START, StateGraph
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolNode

from src.agent.tools.env import (
    search_docs,
    update_skill,
)
from src.agent.utils.config import Config
from src.agent.utils.llm import get_model
from src.generators.benchmark.linter import format_linter_report, run_linter
from src.generators.benchmark.prompts import (
    CODER_PROMPT,
    CRITIC_PROMPT,
    FIXER_PROMPT,
    PLANNER_PROMPT,
)
from src.generators.benchmark.validator import validate_mjcf

# Load config
GEN_CONFIG_PATH = Path(__file__).parent / "generator_config.yaml"
with GEN_CONFIG_PATH.open("r") as f:
    gen_config = yaml.safe_load(f)

MAX_ATTEMPTS = gen_config.get("max_attempts", 10)

CAD_TEMPLATE = """
# Core build123d components
from build123d import (
    Box, Cylinder, Sphere, Torus, Cone, Wedge, 
    Compound, Solid, Part, Location, Rotation, Vector, Axis, Plane,
    Mode, Align, Unit, Shell
)

# Common build123d operations
from build123d import (
    fillet, chamfer, split, mirror, scale, 
    extrude, revolve, loft, sweep, offset
)

# standard builders (Use BuildPart for CSG)
from build123d import BuildPart, BuildSketch, BuildLine

import math
import random
"""


# Define State
class GeneratorState(TypedDict):
    request: str
    plan: str | None
    planner_reasoning: str | None
    code: str | None
    coder_reasoning: str | None
    errors: str | None
    mjcf: str | None
    attempts: int
    validation_passed: bool
    linting_failed: bool
    full_history: list[dict[str, any]]
    messages: Annotated[list[AnyMessage], add_messages]


# Nodes
def planner_node(state: GeneratorState) -> dict[str, any]:
    """Generates a plan based on the user request."""
    print(f"DEBUG: Entering planner_node, request: {state.get('request')}")

    # If plan is already provided, just wrap it in a HumanMessage for consistency if needed,
    # but we can also just return it.
    if state.get("plan"):
        print("DEBUG: planner_node - plan provided externally")
        return {
            "plan": state["plan"],
            "planner_reasoning": "Plan provided externally.",
            "attempts": 0,
            "validation_passed": False,
            "linting_failed": False,
        }

    model = get_model(Config.LLM_MODEL).bind_tools([search_docs, update_skill])

    # Build messages if not present
    messages = state.get("messages")
    if not messages:
        messages = [
            SystemMessage(
                content=PLANNER_PROMPT.format(request=state["request"])
                + "\n\nPlease think step-by-step before providing the plan. "
                "Wrap your internal reasoning in <reasoning> tags and the final plan in <plan> tags. "
                "If you are unsure about build123d syntax or available operations, use the 'search_docs' tool."
            ),
            HumanMessage(content="Create the plan."),
        ]

    response = model.invoke(messages)

    # Extract plan if provided in this turn (non-tool call)
    res_dict = {"messages": [response]}
    content = response.content
    if not response.tool_calls:
        if "<reasoning>" in content and "</reasoning>" in content:
            res_dict["planner_reasoning"] = (
                content.split("<reasoning>")[1].split("</reasoning>")[0].strip()
            )
        if "<plan>" in content and "</plan>" in content:
            res_dict["plan"] = content.split("<plan>")[1].split("</plan>")[0].strip()
        else:
            res_dict["plan"] = content

    return res_dict


def coder_node(state: GeneratorState) -> dict[str, any]:
    """Generates or fixes code based on plan and errors."""
    attempts = state.get("attempts", 0)
    print(f"DEBUG: Entering coder_node, current state attempts: {attempts}")
    model = get_model(Config.LLM_MODEL).bind_tools([search_docs, update_skill])

    errors = state.get("errors")
    code = state.get("code")
    plan = state.get("plan")

    messages = state.get("messages")

    # If the last message is not a ToolMessage and we don't have code yet,
    # it means we just came from the planner and need to start the coder conversation.
    is_starting = not code and (
        not messages or not hasattr(messages[-1], "tool_call_id")
    )
    is_retrying = (
        errors and code and (not messages or not hasattr(messages[-1], "tool_call_id"))
    )

    if is_starting or is_retrying:
        extra_instructions = (
            "\n\nPlease think step-by-step before providing the code. "
            "Wrap your internal reasoning in <reasoning> tags and the final code in <python_code> tags. "
            "If you are unsure about build123d syntax (selectors, context managers like 'with Locations'), use 'search_docs' tool."
        )

        if attempts > 5:
            extra_instructions += (
                "\n\nCRITICAL: You have failed more than 5 times. "
                "Please read `@file:.agent/skills/build123d_cad_drafting_skill/SKILL.md` for guidance. "
                "You should also use the `update_skill` tool to record any new insights, "
                "recurring patterns, or fixes you've discovered to help future attempts."
            )

        if errors and code:
            # Retry mode: use Critic prompt logic
            full_prompt = CRITIC_PROMPT.format(error=errors, code=code)
            messages = [
                SystemMessage(
                    content=FIXER_PROMPT
                    + f"\n\nIMPORTANT: Your code will be prepended with this template:\n{CAD_TEMPLATE}\n"
                    + extra_instructions
                ),
                HumanMessage(content=full_prompt),
            ]
        else:
            # Initial generation
            messages = [
                SystemMessage(
                    content=CODER_PROMPT.format(plan=plan, errors="None")
                    + f"\n\nIMPORTANT: Start from this template:\n{CAD_TEMPLATE}\n"
                    + extra_instructions
                ),
                HumanMessage(content="Generate the code."),
            ]

    response = model.invoke(messages)

    res_dict = {"messages": [response]}

    if not response.tool_calls:
        content = response.content
        reasoning = ""
        if "<reasoning>" in content and "</reasoning>" in content:
            reasoning = content.split("<reasoning>")[1].split("</reasoning>")[0].strip()

        # Extract code
        raw_content = content
        if "<python_code>" in content and "</python_code>" in content:
            raw_content = (
                content.split("<python_code>")[1].split("</python_code>")[0].strip()
            )

        cleaned_code = raw_content
        if "```python" in raw_content:
            cleaned_code = raw_content.split("```python")[1].split("```")[0].strip()
        elif "```" in raw_content:
            cleaned_code = raw_content.split("```")[1].split("```")[0].strip()

        if (
            "import build123d" not in cleaned_code
            and "from build123d" not in cleaned_code
        ):
            cleaned_code = CAD_TEMPLATE + "\n" + cleaned_code

        res_dict.update(
            {
                "code": cleaned_code,
                "coder_reasoning": reasoning,
                "attempts": state.get("attempts", 0)
                + (1 if is_retrying or not state.get("code") else 0),
                "validation_passed": False,
                "linting_failed": False,
                "errors": None,
            }
        )

    return res_dict


def linter_node(state: GeneratorState) -> dict[str, any]:
    """Runs static analysis (ruff, pyrefly) on the provided code."""
    print("DEBUG: Entering linter_node")
    code = state["code"]

    lint_issues = run_linter(code)
    if lint_issues:
        error_msg = format_linter_report(lint_issues)
        print(f"DEBUG: linter_node failed: {error_msg}")
        return {
            "errors": error_msg,
            "validation_passed": False,
            "linting_failed": True,
        }

    print("DEBUG: linter_node passed")
    return {"errors": None, "validation_passed": False, "linting_failed": False}


def validator_node(state: GeneratorState) -> dict[str, any]:
    """Executes code and runs validation."""
    # ... (existing validator_node content)


def skill_populator_node(state: GeneratorState) -> dict[str, any]:
    """Populates the skill with insights after a definitive failure."""
    print("DEBUG: Entering skill_populator_node")
    if state.get("validation_passed"):
        return {}

    model = get_model(Config.LLM_MODEL).bind_tools([update_skill])

    # Collect errors and history
    history = state.get("full_history", [])
    history_str = "\n".join(
        [f"Attempt {h['attempt']}: {h['error_message']}" for h in history]
    )

    prompt = (
        "The benchmark generation has failed definitively after multiple attempts.\n"
        f"Original Request: {state['request']}\n"
        f"Failure History:\n{history_str}\n\n"
        "Your task is to analyze these failures and update the 'build123d_cad_drafting_skill' "
        "with new insights, common pitfalls to avoid, or better patterns. "
        "Use the `update_skill` tool to add a new reference file (e.g., 'lessons_learned.md') "
        "to the skill, describing the issue and the solution or workaround."
    )

    messages = [
        SystemMessage(content=prompt),
        HumanMessage(content="Populate the skill with lessons learned."),
    ]
    model.invoke(messages)

    return {}


# Graph Construction
# Graph Construction
workflow = StateGraph(GeneratorState)

workflow.add_node("planner", planner_node)

workflow.add_node("coder", coder_node)

workflow.add_node("linter", linter_node)

workflow.add_node("validator", validator_node)

workflow.add_node("skill_populator", skill_populator_node)

workflow.add_node("tools", ToolNode([search_docs, update_skill]))


workflow.add_edge(START, "planner")


# ... (router logic)


def should_continue_val(
    state: GeneratorState,
) -> Literal["coder", "skill_populator", END]:
    if state.get("validation_passed") is True:
        print("DEBUG: should_continue_val -> END")

        return END

    if state.get("attempts", 0) >= MAX_ATTEMPTS:
        print("DEBUG: should_continue_val -> skill_populator")

        return "skill_populator"

    print("DEBUG: should_continue_val -> coder")

    return "coder"


workflow.add_conditional_edges("validator", should_continue_val)

workflow.add_edge("skill_populator", END)


generator_agent = workflow.compile()
