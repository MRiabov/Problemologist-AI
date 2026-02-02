import traceback
from typing import TypedDict, Optional, Literal, Dict, Any, List
from langchain_core.messages import HumanMessage, SystemMessage
from langgraph.graph import StateGraph, END

from src.agent.utils.llm import get_model
from src.agent.utils.config import Config
from src.generators.benchmark.prompts import PLANNER_PROMPT, CODER_PROMPT, CRITIC_PROMPT, FIXER_PROMPT
from src.generators.benchmark.validator import validate_mjcf


CAD_TEMPLATE = """from build123d import *
import build123d as bd
import math
import random

def build(seed: int = 0, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> str:
    '''
    Generates a MuJoCo MJCF XML string for a benchmark scenario.
    '''
    random.seed(seed)
"""

# Define State
class GeneratorState(TypedDict):
    request: str
    plan: Optional[str]
    code: Optional[str]
    errors: Optional[str]
    mjcf: Optional[str]
    attempts: int
    validation_passed: bool


# Nodes
def planner_node(state: GeneratorState) -> Dict[str, Any]:
    """Generates a plan based on the user request."""
    # print(f"--- PLANNER NODE (Attempt {state.get('attempts', 0)}) ---")
    model = get_model(Config.LLM_MODEL)

    messages = [
        SystemMessage(content=PLANNER_PROMPT.format(request=state["request"])),
        HumanMessage(content="Create the plan."),
    ]

    response = model.invoke(messages)
    return {"plan": response.content, "attempts": 0, "validation_passed": False}


def coder_node(state: GeneratorState) -> Dict[str, Any]:
    """Generates or fixes code based on plan and errors."""
    # print(f"--- CODER NODE (Attempt {state.get('attempts', 0)}) ---")
    model = get_model(Config.LLM_MODEL)

    errors = state.get("errors")
    code = state.get("code")
    plan = state.get("plan")

    if errors and code:
        # Retry mode: use Critic prompt logic
        full_prompt = CRITIC_PROMPT.format(error=errors, code=code)
        messages = [
            SystemMessage(content=FIXER_PROMPT + f"\n\nIMPORTANT: Your code will be prepended with this template, do not redefine it unless necessary:\n{CAD_TEMPLATE}"),
            HumanMessage(content=full_prompt),
        ]
    else:
        # Initial generation
        messages = [
            SystemMessage(content=CODER_PROMPT.format(plan=plan, errors="None") + f"\n\nIMPORTANT: Start from this template. You only need to provide the implementation inside the build function or additional helper functions:\n{CAD_TEMPLATE}"),
            HumanMessage(content="Generate the code."),
        ]

    response = model.invoke(messages)

    # Extract code
    raw_content = response.content
    cleaned_code = raw_content
    if "```python" in raw_content:
        cleaned_code = raw_content.split("```python")[1].split("```")[0].strip()
    elif "```" in raw_content:
        cleaned_code = raw_content.split("```")[1].split("```")[0].strip()

    if "from build123d import *" not in cleaned_code:
        cleaned_code = CAD_TEMPLATE + "\n" + cleaned_code

    return {"code": cleaned_code, "attempts": state.get("attempts", 0) + 1}


def validator_node(state: GeneratorState) -> Dict[str, Any]:
    """Executes code and runs validation."""
    # print("--- VALIDATOR NODE ---")
    code = state["code"]
    
    # Prepend template if not already present (or just always prepend for safety)
    full_code = code
    if "from build123d import *" not in code:
        full_code = CAD_TEMPLATE + "\n" + code

    try:
        from src.generators.benchmark.manager import execute_build
        # Call build with seed 0 and default scale (1,1,1) for base validation
        mjcf_xml = execute_build(full_code, 0, scale=(1.0, 1.0, 1.0))

        if not isinstance(mjcf_xml, str):
            return {
                "errors": f"Function 'build' returned {type(mjcf_xml)}, expected str (MJCF XML).",
                "validation_passed": False,
            }

        # Validate MJCF
        report = validate_mjcf(mjcf_xml)

        if report["is_valid"]:
            return {"validation_passed": True, "mjcf": mjcf_xml, "errors": None}
        else:
            return {
                "validation_passed": False,
                "errors": f"Validation failed: {report['error_message']}",
            }

    except Exception as e:
        return {
            "errors": f"Syntax/Runtime Error: {e}\n{traceback.format_exc()}",
            "validation_passed": False,
        }


def should_continue(state: GeneratorState) -> Literal["coder", END]:
    """Decides whether to retry or end."""
    if state["validation_passed"]:
        return END
    if state["attempts"] >= 3:
        return END
    return "coder"


# Graph Construction
workflow = StateGraph(GeneratorState)

workflow.add_node("planner", planner_node)
workflow.add_node("coder", coder_node)
workflow.add_node("validator", validator_node)

workflow.set_entry_point("planner")
workflow.add_edge("planner", "coder")
workflow.add_edge("coder", "validator")
workflow.add_conditional_edges("validator", should_continue)

generator_agent = workflow.compile()
