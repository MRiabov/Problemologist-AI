import traceback
from typing import TypedDict, Optional, Literal, Dict, Any, List
from pathlib import Path
import yaml
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.graph import END, StateGraph

from src.agent.utils.config import Config
from src.agent.utils.llm import get_model
from src.generators.benchmark.linter import run_linter
from src.generators.benchmark.prompts import (
    CODER_PROMPT,
    CRITIC_PROMPT,
    FIXER_PROMPT,
    LINTER_FEEDBACK_PREFIX,
    PLANNER_PROMPT,
)
from src.generators.benchmark.validator import validate_mjcf

# Load config
GEN_CONFIG_PATH = Path(__file__).parent / "generator_config.yaml"
with GEN_CONFIG_PATH.open("r") as f:
    gen_config = yaml.safe_load(f)

MAX_ATTEMPTS = gen_config.get("max_attempts", 10)

CAD_TEMPLATE = """from build123d import *
import build123d as bd
import math
import random

# Common build123d patterns for the model to follow
# 1. Use Box() inside BuildPart() context, not BuildBox.
# 2. Use Compound(children=list_of_solids) to combine parts.
# 3. Use scale(objects, by=(sx, sy, sz)) for non-uniform scaling.
# 4. Use part.translate((x, y, z)) or part.move(Location((x,y,z))). 
#    CRITICAL: There is NO 'bd.move(part, ...)' function. Use methods on the part itself.
"""


# Define State
class GeneratorState(TypedDict):
    request: str
    plan: Optional[str]
    planner_reasoning: Optional[str]
    code: Optional[str]
    coder_reasoning: Optional[str]
    errors: Optional[str]
    full_history: Optional[List[Dict[str, Any]]]  # List of reports from all attempts
    mjcf: Optional[str]
    attempts: int
    validation_passed: bool
    linting_failed: bool


# Nodes
def planner_node(state: GeneratorState) -> Dict[str, Any]:
    """Generates a plan based on the user request."""
    # print(f"--- PLANNER NODE (Attempt {state.get('attempts', 0)}) ---")
    model = get_model(Config.LLM_MODEL)

    messages = [
        SystemMessage(
            content=PLANNER_PROMPT.format(request=state["request"]) 
            + "\n\nPlease think step-by-step before providing the plan. "
            "Wrap your internal reasoning in <reasoning> tags and the final plan in <plan> tags."
        ),
        HumanMessage(content="Create the plan."),
    ]

    response = model.invoke(messages)
    content = response.content

    reasoning = ""
    plan = content

    if "<reasoning>" in content and "</reasoning>" in content:
        reasoning = content.split("<reasoning>")[1].split("</reasoning>")[0].strip()

    if "<plan>" in content and "</plan>" in content:
        plan = content.split("<plan>")[1].split("</plan>")[0].strip()
    
    return {
        "plan": plan,
        "planner_reasoning": reasoning,
        "attempts": 0,
        "validation_passed": False,
    }


def coder_node(state: GeneratorState) -> Dict[str, Any]:
    """Generates or fixes code based on plan and errors."""
    # print(f"--- CODER NODE (Attempt {state.get('attempts', 0)}) ---")
    model = get_model(Config.LLM_MODEL)

    errors = state.get("errors")
    code = state.get("code")
    plan = state.get("plan")

    extra_instructions = (
        "\n\nPlease think step-by-step before providing the code. "
        "Wrap your internal reasoning in <reasoning> tags and the final code in <python_code> tags."
    )

    if errors and code:
        # Retry mode: use Critic prompt logic
        full_prompt = CRITIC_PROMPT.format(error=errors, code=code)
        messages = [
            SystemMessage(
                content=FIXER_PROMPT
                + f"\n\nIMPORTANT: Your code will be prepended with this template, do not redefine it unless necessary:\n{CAD_TEMPLATE}"
                + extra_instructions
            ),
            HumanMessage(content=full_prompt),
        ]
    else:
        # Initial generation
        messages = [
            SystemMessage(
                content=CODER_PROMPT.format(plan=plan, errors="None")
                + f"\n\nIMPORTANT: Start from this template. You only need to provide the implementation inside the build function or additional helper functions:\n{CAD_TEMPLATE}"
                + extra_instructions
            ),
            HumanMessage(content="Generate the code."),
        ]

    response = model.invoke(messages)
    content = response.content

    reasoning = ""
    if "<reasoning>" in content and "</reasoning>" in content:
        reasoning = content.split("<reasoning>")[1].split("</reasoning>")[0].strip()

    # Extract code
    raw_content = content
    if "<python_code>" in content and "</python_code>" in content:
        raw_content = content.split("<python_code>")[1].split("</python_code>")[0].strip()
    
    cleaned_code = raw_content
    if "```python" in raw_content:
        cleaned_code = raw_content.split("```python")[1].split("```")[0].strip()
    elif "```" in raw_content:
        cleaned_code = raw_content.split("```")[1].split("```")[0].strip()

    if "from build123d import *" not in cleaned_code:
        cleaned_code = CAD_TEMPLATE + "\n" + cleaned_code

    return {
        "code": cleaned_code,
        "coder_reasoning": reasoning,
        "attempts": state.get("attempts", 0) + 1,
    }


def linter_node(state: GeneratorState) -> dict[str, any]:
    """Runs static analysis (ruff, pyrefly) on the generated code."""
    code = state["code"]
    errors = []

    # Heuristic check for common build123d attribute hallucinations
    common_hallucinations = {
        ".scaled(": "'component.scaled' is not a valid attribute in build123d. Use 'scale(part, ...)' instead.",
        # Add more here if needed (e.g., .translated(, .rotated() if they cause issues)
    }

    for marker, tip in common_hallucinations.items():
        if marker in code:
            errors.append(f"[Heuristic] {tip}")

    lint_issues = run_linter(code)
    errors.extend(lint_issues)

    if errors:
        error_msg = LINTER_FEEDBACK_PREFIX + "\n".join(errors)
        return {
            "errors": error_msg,
            "validation_passed": False,
            "linting_failed": True,
        }

    return {"errors": None, "validation_passed": True, "linting_failed": False}


def validator_node(state: GeneratorState) -> dict[str, any]:
    """Executes code and runs validation."""
    code = state["code"]

    try:
        from src.generators.benchmark.manager import execute_build

        # Call build with seed 0 and default scale (1,1,1) for base validation
        # We use a relative path for assets dir for sandbox compatibility
        rel_temp_assets = ".agent_storage/temp_assets"
        mjcf_xml = execute_build(code, 0, scale_factors=(1.0, 1.0, 1.0), asset_dir=rel_temp_assets)

        if not isinstance(mjcf_xml, str):
            return {
                "errors": f"Function 'build' returned {type(mjcf_xml)}, expected str.",
                "validation_passed": False,
            }

        # Validate MJCF
        report = validate_mjcf(mjcf_xml)
        
        # Track history
        history = state.get("full_history") or []
        history.append(report)

        if report["is_valid"]:
            return {"validation_passed": True, "mjcf": mjcf_xml, "errors": None, "full_history": history}
        else:
            return {
                "validation_passed": False,
                "errors": f"Validation failed: {report['error_message']}",
                "full_history": history
            }

    except Exception as e:
        error_msg = f"Syntax/Runtime Error: {e}\n{traceback.format_exc()}"
        history = state.get("full_history") or []
        history.append({"is_valid": False, "error_message": error_msg})
        return {
            "errors": error_msg,
            "validation_passed": False,
            "full_history": history
        }


# Graph Construction
workflow = StateGraph(GeneratorState)

workflow.add_node("planner", planner_node)
workflow.add_node("coder", coder_node)
workflow.add_node("linter", linter_node)
workflow.add_node("validator", validator_node)

workflow.set_entry_point("planner")
workflow.add_edge("planner", "coder")
workflow.add_edge("coder", "linter")


def should_continue_lint(state: GeneratorState) -> Literal["coder", "validator"]:
    """Decides whether to retry after linting or proceed to simulation."""
    if state["linting_failed"] and state["attempts"] < MAX_ATTEMPTS:
        return "coder"
    return "validator"


workflow.add_conditional_edges("linter", should_continue_lint)
workflow.add_edge("validator", "should_continue_proxy")


def should_continue(state: GeneratorState) -> Literal["coder", END]:
    """Decides whether to retry after validation or end."""
    if state["validation_passed"]:
        return END
    if state["attempts"] >= MAX_ATTEMPTS:
        return END
    return "coder"


workflow.add_node("should_continue_proxy", lambda x: x)
workflow.add_conditional_edges("should_continue_proxy", should_continue)

generator_agent = workflow.compile()
