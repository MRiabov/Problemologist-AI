import asyncio
import base64
import os
import re

import httpx
import structlog
import yaml
import dspy
from langchain_core.messages import AIMessage

from controller.agent.nodes.base import SharedNodeContext
from controller.agent.dspy_utils import init_dspy, wrap_tool_for_dspy
from controller.agent.signatures import (
    BenchmarkPlannerSignature,
    BenchmarkCoderSignature,
    BenchmarkReviewerSignature
)
from shared.simulation.schemas import (
    RandomizationStrategy,
    SimulatorBackendType,
)

from ..config import settings
from .state import BenchmarkGeneratorState
from .tools import get_benchmark_tools

logger = structlog.get_logger(__name__)

OBJECTIVES_FILE = "objectives.yaml"
SCRIPT_FILE = "script.py"


def extract_python_code(text: str) -> str:
    """Extracts python code block from markdown."""
    pattern = r"```python\n(.*?)\n```"
    match = re.search(pattern, text, re.DOTALL)
    if match:
        return match.group(1).strip()
    return text.strip()


async def planner_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Breaks down the user prompt into a randomization strategy using DSPy CodeAct.
    """
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    init_dspy(session_id=session_id)
    logger.info("planner_node_start", session_id=session_id)

    # Init Git
    await ctx.worker_client.git_init()

    # Custom Objectives Logic
    custom_objectives = state.session.custom_objectives
    if custom_objectives:
        if await ctx.worker_client.exists(OBJECTIVES_FILE):
            try:
                obj_content = await ctx.worker_client.read_file(OBJECTIVES_FILE)
                obj_data = yaml.safe_load(obj_content)
                if not isinstance(obj_data, dict): obj_data = {}
                if "constraints" not in obj_data: obj_data["constraints"] = {}
                if custom_objectives.max_unit_cost is not None:
                    obj_data["constraints"]["max_unit_cost"] = (
                        custom_objectives.max_unit_cost
                    )
                if custom_objectives.max_weight is not None:
                    obj_data["constraints"]["max_weight"] = (
                        custom_objectives.max_weight
                    )
                if custom_objectives.target_quantity is not None:
                    obj_data["constraints"]["target_quantity"] = (
                        custom_objectives.target_quantity
                    )

                new_content = yaml.dump(obj_data, sort_keys=False)
                await ctx.worker_client.write_file(OBJECTIVES_FILE, new_content)
            except Exception as e:
                logger.warning("planner_objectives_update_failed", error=str(e))

    # Use CodeAct for planning
    raw_tools = get_benchmark_tools(ctx.fs, session_id)
    tools = [wrap_tool_for_dspy(t) for t in raw_tools]
    planner = dspy.CodeAct(BenchmarkPlannerSignature, tools=tools)

    try:
        hist = state.messages or []
        history_str = "\n".join([f"{m.type}: {m.content}" for m in hist])
        result = planner(prompt=state.session.prompt, history=history_str)

        plan = result.plan
        if isinstance(plan, dict):
            plan = RandomizationStrategy(**plan)
        elif not isinstance(plan, RandomizationStrategy):
             plan = RandomizationStrategy(
                 theme="General", reasoning="Extracted from CodeAct"
             )

        state.plan = plan
        state.messages.append(AIMessage(content=(
            f"Generated randomization strategy using DSPy CodeAct: {plan.theme}"
        )))

    except Exception as e:
        logger.error("planner_dspy_run_failed", error=str(e), session_id=session_id)
        state.plan = RandomizationStrategy(theme="error", reasoning=str(e))

    return state


async def coder_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Generates build123d script using DSPy CodeAct.
    """
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    init_dspy(session_id=session_id)
    logger.info("coder_node_start", session_id=session_id)

    validation_logs = "\n".join(state.session.validation_logs)
    if state.simulation_result and not state.simulation_result.valid:
        validation_logs += "\n" + "\n".join(state.simulation_result.logs)

    objectives_yaml = "# No objectives.yaml found."
    try:
        if await ctx.worker_client.exists(OBJECTIVES_FILE):
            objectives_yaml = await ctx.worker_client.read_file(OBJECTIVES_FILE)
    except Exception: pass

    # Setup DSPy CodeAct
    raw_tools = get_benchmark_tools(ctx.fs, session_id)
    tools = [wrap_tool_for_dspy(t) for t in raw_tools]
    coder = dspy.CodeAct(BenchmarkCoderSignature, tools=tools)

    try:
        coder(
            prompt=state.session.prompt,
            plan=state.plan.model_dump_json() if state.plan else "None",
            objectives=objectives_yaml,
            feedback=state.review_feedback or "No feedback provided.",
            logs=validation_logs
        )
        state.messages.append(AIMessage(
            content="Benchmark implementation completed using DSPy CodeAct."
        ))
    except Exception as e:
        logger.error("coder_dspy_failed", error=str(e))

    # Retrieve script
    try:
        state.current_script = await ctx.worker_client.read_file(SCRIPT_FILE)
    except Exception: pass

    # Validation and Verification
    if state.current_script:
        from worker.utils.file_validation import validate_node_output
        is_valid, errors = validate_node_output(
            "coder", {SCRIPT_FILE: state.current_script}
        )
        if not is_valid:
            state.session.validation_logs.append(f"Output validation failed: {errors}")

        try:
            val_res = await ctx.worker_client.validate(script_path=SCRIPT_FILE)
            if not val_res.success:
                from shared.simulation.schemas import ValidationResult
                state.simulation_result = ValidationResult(
                    valid=False, cost=0,
                    logs=[f"Geometric validation failed: {val_res.message}"],
                    render_paths=[], render_data=[]
                )
            else:
                backend = SimulatorBackendType.MUJOCO
                try:
                    if objectives_yaml and not objectives_yaml.startswith("#"):
                        obj_data = yaml.safe_load(objectives_yaml)
                        if (obj_data and "physics" in obj_data and
                            "backend" in obj_data["physics"]):
                            backend = SimulatorBackendType(
                                obj_data["physics"]["backend"]
                            )
                except Exception: pass

                sim_res = await ctx.worker_client.simulate(
                    script_path=SCRIPT_FILE, backend=backend
                )
                if not sim_res.success:
                    from shared.simulation.schemas import ValidationResult
                    state.simulation_result = ValidationResult(
                        valid=False, cost=0,
                        logs=[f"Physics simulation failed: {sim_res.message}"],
                        render_paths=[], render_data=[]
                    )
                else:
                    r_paths = (sim_res.artifacts.get("render_paths", [])
                               if sim_res.artifacts else [])
                    async def _download(url_path):
                        u = f"{worker_url}/assets/{url_path.lstrip('/')}"
                        async with httpx.AsyncClient() as http_client:
                            r = await http_client.get(
                                u, headers={"X-Session-ID": session_id}
                            )
                            return r.content if r.status_code == 200 else None
                    results = await asyncio.gather(*[_download(p) for p in r_paths])
                    from shared.simulation.schemas import ValidationResult
                    state.simulation_result = ValidationResult(
                        valid=True, cost=0, logs=["Validation passed."],
                        render_paths=r_paths,
                        render_data=[r for r in results if r is not None]
                    )
        except Exception as e:
            logger.error("integrated_validation_error", error=str(e))

    return state


async def cots_search_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")
    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    init_dspy(session_id=session_id)
    raw_tools = get_benchmark_tools(ctx.fs, session_id)
    tools = [wrap_tool_for_dspy(t) for t in raw_tools]

    class SearchSignature(dspy.Signature):
        """Find components for the benchmark."""
        prompt = dspy.InputField()
        results = dspy.OutputField()

    searcher = dspy.CodeAct(SearchSignature, tools=tools)
    result = searcher(
        prompt=f"Find components for the benchmark: {state.session.prompt}"
    )
    state.messages.append(AIMessage(
        content=f"COTS Search result using DSPy CodeAct: {result.results}"
    ))
    return state


async def skills_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    return state


async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Agentic review of the generated benchmark using DSPy CodeAct.
    """
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    init_dspy(session_id=session_id)
    logger.info("reviewer_node_start", round=state.review_round)

    state.review_round = state.review_round + 1
    current_round = state.review_round
    review_filename = f"reviews/review-round-{current_round}/review.md"

    # Vision inputs
    sim_result = state.simulation_result
    render_data = sim_result.render_data if sim_result else []

    renders_list = []
    for img_bytes in (render_data or [])[:8]:
        try:
            if img_bytes:
                encoded = base64.b64encode(img_bytes).decode("utf-8")
                renders_list.append(f"data:image/png;base64,{encoded}")
        except Exception: pass

    render_info = (
        f"Found {len(render_data)} renders. "
        f"Images (base64): {', '.join(renders_list[:3])}..."
    )

    # Setup Tools
    all_tools = get_benchmark_tools(ctx.fs, session_id)

    from langchain_core.tools import tool as lc_tool
    @lc_tool
    async def write_review_file(path: str, content: str) -> str:
        """Write the review to the review file."""
        if path.lstrip("/") != review_filename.lstrip("/"):
            return "Error: Unauthorized path."
        success = await ctx.worker_client.write_file(path, content)
        return "Review written successfully." if success else "Error writing review."

    safe_raw_tools = [
        t for t in all_tools
        if t.name not in ("write_file", "edit_file", "submit_for_review")
    ]
    safe_raw_tools.append(write_review_file)
    tools = [wrap_tool_for_dspy(t) for t in safe_raw_tools]

    # DSPy CodeAct Reviewer
    reviewer = dspy.CodeAct(BenchmarkReviewerSignature, tools=tools)

    try:
        result = reviewer(
            theme=state.plan.theme if state.plan else "Unknown",
            prompt=state.session.prompt,
            renders=render_info,
            script=state.current_script
        )

        review_parsed = result.review_result
        state.review_feedback = f"{review_parsed.decision}: {review_parsed.reason}"
        if review_parsed.required_fixes:
            state.review_feedback += (
                "\nFixes: " + ", ".join(review_parsed.required_fixes)
            )

        state.messages.append(AIMessage(
            content=f"Benchmark review completed. Decision: {review_parsed.decision}"
        ))

    except Exception as e:
        state.review_feedback = f"Error executing reviewer: {e}"

    return state
