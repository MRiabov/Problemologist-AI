import traceback
from pathlib import Path
from typing import Annotated, Literal, TypedDict

import yaml
from langchain_core.messages import AnyMessage, HumanMessage, SystemMessage
from langgraph.graph import END, START, StateGraph
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolNode

from src.agent.tools.env import (
    init_skill,
    list_skill_files,
    list_skills,
    package_skill,
    read_skill,
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

ALL_TOOLS = [
    init_skill,
    list_skill_files,
    list_skills,
    package_skill,
    read_skill,
    search_docs,
    update_skill,
]

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

    if state.get("plan"):
        return {
            "plan": state["plan"],
            "planner_reasoning": "Plan provided externally.",
            "attempts": 0,
            "validation_passed": False,
            "linting_failed": False,
        }

    model = get_model(Config.LLM_MODEL).bind_tools(
        [
            search_docs,
            update_skill,
            read_skill,
            list_skills,
            list_skill_files,
            init_skill,
            package_skill,
        ]
    )

    # Build messages if not present
    messages = state.get("messages")
    if not messages:
        system_msg = SystemMessage(content=PLANNER_PROMPT)
        # Mandatory skill check instruction
        system_msg.content += (
            "\n\nMANDATORY: Before planning any `build123d` implementation, "
            "you MUST use `read_skill` to read `build123d_cad_drafting_skill` (SKILL.md). "
            "It contains expert knowledge, curated patterns, and critical pitfalls.\n"
            "Use `list_skills` and `list_skill_files` to discover other knowledge."
        )
        messages = [system_msg, HumanMessage(content=state["request"])]

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
    model = get_model(Config.LLM_MODEL).bind_tools(
        [
            search_docs,
            update_skill,
            read_skill,
            list_skills,
            list_skill_files,
            init_skill,
            package_skill,
        ]
    )

    errors = state.get("errors")
    code = state.get("code")
    plan = state.get("plan")
    messages = state.get("messages")

    is_starting = not code and (
        not messages or not hasattr(messages[-1], "tool_call_id")
    )
    is_retrying = (
        errors and code and (not messages or not hasattr(messages[-1], "tool_call_id"))
    )

    if is_starting or is_retrying:
        extra_instructions = (
            "\n\nPlease think step-by-step. Wrap reasoning in <reasoning> tags "
            "and code in <python_code> tags.\n"
            "MANDATORY: Use `read_skill` to read `build123d_cad_drafting_skill` (SKILL.md) "
            "before writing code."
        )

        if attempts > 5:
            extra_instructions += (
                "\n\nCRITICAL: Multiple failures occurred. "
                "Consult the skill documentation thoroughly. "
                "Record any new insights or fixes using `update_skill`."
            )

        if errors and code:
            full_prompt = CRITIC_PROMPT.format(error=errors, code=code)
            messages = [
                SystemMessage(
                    content=FIXER_PROMPT
                    + f"\n\nIMPORTANT: Template:\n{CAD_TEMPLATE}\n"
                    + extra_instructions
                ),
                HumanMessage(content=full_prompt),
            ]
        else:
            messages = [
                SystemMessage(
                    content=CODER_PROMPT.format(plan=plan, errors="None")
                    + f"\n\nIMPORTANT: Template:\n{CAD_TEMPLATE}\n"
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
                "attempts": attempts + (1 if is_retrying or not code else 0),
                "validation_passed": False,
                "linting_failed": False,
                "errors": None,
            }
        )

    return res_dict


def linter_node(state: GeneratorState) -> dict[str, any]:
    """Runs static analysis (ruff, pyrefly) on the provided code."""
    code = state["code"]
    lint_issues = run_linter(code)
    if lint_issues:
        error_msg = format_linter_report(lint_issues)
        return {
            "errors": error_msg,
            "validation_passed": False,
            "linting_failed": True,
        }
    return {"errors": None, "validation_passed": False, "linting_failed": False}


def validator_node(state: GeneratorState) -> dict[str, any]:
    """Executes code and runs validation."""
    code = state["code"]
    from src.generators.benchmark.manager import execute_build

    try:
        mjcf_xml = execute_build(code, seed=0)
        report = validate_mjcf(mjcf_xml)

        if report["is_valid"]:
            return {
                "validation_passed": True,
                "mjcf": mjcf_xml,
                "errors": None,
                "linting_failed": False,
            }

        history = state.get("full_history", [])
        history.append(
            {
                "attempt": state["attempts"],
                "error_message": report["error_message"],
                "code": code,
            }
        )
        return {
            "validation_passed": False,
            "errors": report["error_message"],
            "full_history": history,
        }
    except Exception as e:
        error_msg = traceback.format_exc()
        history = state.get("full_history", [])
        history.append(
            {"attempt": state["attempts"], "error_message": error_msg, "code": code}
        )
        return {
            "validation_passed": False,
            "errors": f"Execution or validation error: {e!s}",
            "full_history": history,
        }


def skill_populator_node(state: GeneratorState) -> dict[str, any]:
    """Populates the skill with insights after a definitive failure."""
    if state.get("validation_passed"):
        return {}

    model = get_model(Config.LLM_MODEL).bind_tools(
        [
            update_skill,
            read_skill,
            list_skills,
            list_skill_files,
            init_skill,
            package_skill,
        ]
    )

    history = state.get("full_history", [])
    history_str = "\n".join(
        [f"Attempt {h['attempt']}: {h['error_message']}" for h in history]
    )

    prompt = (
        "The benchmark generation has failed definitively.\n"
        f"Original Request: {state['request']}\n"
        f"Failure History:\n{history_str}\n\n"
        "Your goal is to extract lessons learned and add them to "
        "`build123d_cad_drafting_skill` using canonical structure:\n"
        "1. Use `read_skill` to see current documentation.\n"
        "2. Add detailed insights to `references/` (e.g., 'common_failures_mjcf.md').\n"
        "3. (Optional) If you found a recurring code bug, add a Python helper to `scripts/`.\n"
        "4. Update `SKILL.md` to link to these new resources.\n"
        "5. Finally, use `package_skill` to validate and finalize changes."
    )

    messages = [
        SystemMessage(content=prompt),
        HumanMessage(content="Update the skill with insights from these failures."),
    ]
    model.invoke(messages)

    return {}


# Graph Construction
workflow = StateGraph(GeneratorState)

workflow.add_node("planner", planner_node)
workflow.add_node("coder", coder_node)
workflow.add_node("linter", linter_node)
workflow.add_node("validator", validator_node)
workflow.add_node("skill_populator", skill_populator_node)

# Tool Nodes
workflow.add_node("planner_tools", ToolNode(ALL_TOOLS))
workflow.add_node("coder_tools", ToolNode(ALL_TOOLS))

workflow.add_edge(START, "planner")


# Conditional logic for Planner
def should_continue_planner(state: GeneratorState) -> Literal["planner_tools", "coder"]:
    messages = state.get("messages", [])
    if messages and hasattr(messages[-1], "tool_calls") and messages[-1].tool_calls:
        return "planner_tools"
    return "coder"


workflow.add_conditional_edges("planner", should_continue_planner)
workflow.add_edge("planner_tools", "planner")


# Conditional logic for Coder
def should_continue_coder(state: GeneratorState) -> Literal["coder_tools", "linter"]:
    messages = state.get("messages", [])
    if messages and hasattr(messages[-1], "tool_calls") and messages[-1].tool_calls:
        return "coder_tools"
    return "linter"


workflow.add_conditional_edges("coder", should_continue_coder)
workflow.add_edge("coder_tools", "coder")


def should_continue_lint(state: GeneratorState) -> Literal["coder", "validator"]:
    if state.get("linting_failed"):
        return "coder"
    return "validator"


workflow.add_conditional_edges("linter", should_continue_lint)


def should_continue_val(
    state: GeneratorState,
) -> Literal["coder", "skill_populator", END]:
    if state.get("validation_passed"):
        return END

    if state.get("attempts", 0) >= MAX_ATTEMPTS:
        return "skill_populator"

    return "coder"


workflow.add_conditional_edges("validator", should_continue_val)
workflow.add_edge("skill_populator", END)

app = workflow.compile()
generator_agent = app  # Alias for compatibility with tests and manager
