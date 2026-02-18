import asyncio
import os
import re
from contextlib import suppress

import dspy
import httpx
import structlog
import yaml
from langchain_core.messages import AIMessage, HumanMessage

from controller.agent.nodes.base import SharedNodeContext
from controller.agent.nodes.reviewer import ReviewResult
from shared.enums import SessionStatus
from shared.simulation.schemas import (
    RandomizationStrategy,
    SimulatorBackendType,
)

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


class BenchmarkPlannerSignature(dspy.Signature):
    """
    Planner node: Analyzes the user prompt and creates a randomization strategy.
    You must use the provided tools to set up the objectives if necessary.
    When done, use SUBMIT to provide the final randomization strategy.
    """

    prompt = dspy.InputField()
    history = dspy.InputField()
    review_feedback = dspy.InputField()
    plan: RandomizationStrategy = dspy.OutputField()


async def planner_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Breaks down the user prompt into a randomization strategy using DSPy.
    """
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    logger.info("planner_node_start", session_id=session_id)

    # Init Git
    await ctx.worker_client.git_init()

    # Custom Objectives Logic (Restore)
    custom_objectives = state.session.custom_objectives
    if custom_objectives:
        logger.info("planner_updating_objectives", session_id=session_id)
        if await ctx.worker_client.exists(OBJECTIVES_FILE):
            try:
                obj_content = await ctx.worker_client.read_file(OBJECTIVES_FILE)
                obj_data = yaml.safe_load(obj_content)
                if not isinstance(obj_data, dict):
                    obj_data = {}
                if "constraints" not in obj_data:
                    obj_data["constraints"] = {}
                # Update constraints based on custom objectives
                if custom_objectives.max_unit_cost is not None:
                    obj_data["constraints"]["max_unit_cost"] = (
                        custom_objectives.max_unit_cost
                    )
                if custom_objectives.max_weight is not None:
                    obj_data["constraints"]["max_weight"] = custom_objectives.max_weight
                if custom_objectives.target_quantity is not None:
                    obj_data["constraints"]["target_quantity"] = (
                        custom_objectives.target_quantity
                    )

                new_content = yaml.dump(obj_data, sort_keys=False)
                await ctx.worker_client.write_file(OBJECTIVES_FILE, new_content)
                logger.info("planner_objectives_updated", session_id=session_id)
            except Exception as e:
                logger.warning("planner_objectives_update_failed", error=str(e))

    # Setup DSPy Program
    from controller.agent.dspy_utils import WorkerInterpreter

    interpreter = WorkerInterpreter(
        worker_client=ctx.worker_client, session_id=session_id
    )

    # Tools for benchmark generator
    tools = get_benchmark_tools(ctx.fs, session_id)
    tool_functions = {}
    for t in tools:
        func = getattr(t, "func", getattr(t, "_run", None))
        if func:
            tool_functions[t.name] = func

    program = dspy.CodeAct(
        BenchmarkPlannerSignature.with_instructions(ctx.pm.render("benchmark_planner")),
        tools=list(tool_functions.values()),
        interpreter=interpreter,
    )

    try:
        with dspy.settings.context(lm=ctx.dspy_lm):
            logger.info("planner_dspy_invoke_start", session_id=session_id)
            prediction = program(
                prompt=state.session.prompt,
                history=str(state.messages or []),
                review_feedback=(
                    state.review_feedback
                    if state.session.status == SessionStatus.REJECTED
                    else "No feedback yet."
                ),
            )
            logger.info("planner_dspy_invoke_complete", session_id=session_id)

        state.plan = prediction.plan
        state.messages.append(
            HumanMessage(content=f"Generated plan: {state.plan.theme}")
        )

    except Exception as e:
        logger.error("planner_dspy_failed", error=str(e))
        state.plan = RandomizationStrategy(theme="error", reasoning=str(e))
    finally:
        interpreter.shutdown()

    return state


class BenchmarkCoderSignature(dspy.Signature):
    """
    Generates build123d script and validates it for the benchmark.
    You must use the provided tools to create the benchmark script in 'script.py'.
    When done, use SUBMIT to provide a summary of your work.
    """

    prompt = dspy.InputField()
    plan = dspy.InputField()
    objectives_yaml = dspy.InputField()
    review_feedback = dspy.InputField()
    validation_logs = dspy.InputField()
    journal = dspy.OutputField(desc="A summary of what was done")


async def coder_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Generates build123d script and validates it.
    Refactored to use DSPy CodeAct with remote worker execution.
    """
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    logger.info("coder_node_start", session_id=session_id)

    # Context construction
    validation_logs = "\n".join(state.session.validation_logs)
    if state.simulation_result and not state.simulation_result.valid:
        validation_logs += "\n" + "\n".join(state.simulation_result.logs)

    objectives_yaml = "# No objectives.yaml found."
    try:
        if await ctx.worker_client.exists(OBJECTIVES_FILE):
            resp = await ctx.worker_client.read_file(OBJECTIVES_FILE)
            objectives_yaml = resp
    except Exception:
        pass

    # Setup DSPy Program
    from controller.agent.dspy_utils import WorkerInterpreter

    interpreter = WorkerInterpreter(
        worker_client=ctx.worker_client, session_id=session_id
    )

    tools = get_benchmark_tools(ctx.fs, session_id)
    tool_functions = {}
    for t in tools:
        func = getattr(t, "func", getattr(t, "_run", None))
        if func:
            tool_functions[t.name] = func

    program = dspy.CodeAct(
        BenchmarkCoderSignature.with_instructions(ctx.pm.render("benchmark_coder")),
        tools=list(tool_functions.values()),
        interpreter=interpreter,
    )

    # Invoke DSPy Program
    try:
        with dspy.settings.context(lm=ctx.dspy_lm):
            logger.info("coder_dspy_invoke_start", session_id=session_id)
            prediction = program(
                prompt=state.session.prompt,
                plan=state.plan.model_dump_json() if state.plan else "None",
                objectives_yaml=objectives_yaml,
                review_feedback=(
                    state.review_feedback
                    if state.session.status == SessionStatus.REJECTED
                    else "No feedback provided."
                ),
                validation_logs=validation_logs,
            )
            logger.info("coder_dspy_invoke_complete", session_id=session_id)

            new_journal = getattr(prediction, "journal", "No journal provided.")
            state.messages.append(AIMessage(content=f"Work summary: {new_journal}"))
    except Exception as e:
        logger.error("coder_dspy_failed", error=str(e))
    finally:
        interpreter.shutdown()

    # Retrieve script
    import contextlib

    with contextlib.suppress(Exception):
        state.current_script = await ctx.worker_client.read_file(SCRIPT_FILE)

    # Validation Logic (Same as before)
    if state.current_script:
        from worker.utils.file_validation import validate_node_output

        is_valid, errors = validate_node_output(
            "coder", {SCRIPT_FILE: state.current_script}
        )
        if not is_valid:
            logger.warning("benchmark_coder_validation_failed", errors=errors)
            state.session.validation_logs.append(f"Output validation failed: {errors}")

    # Run Verification (Geometric + Physics)
    script = state.current_script
    if script:
        logger.info("running_integrated_validation", session_id=session_id)
        try:
            val_res = await ctx.worker_client.validate(script_path=SCRIPT_FILE)
            if not val_res.success:
                from shared.simulation.schemas import ValidationResult

                state.simulation_result = ValidationResult(
                    valid=False,
                    cost=0,
                    logs=[f"Geometric validation failed: {val_res.message}"],
                    render_paths=[],
                    render_data=[],
                )
            else:
                # physics simulation
                backend = SimulatorBackendType.GENESIS
                try:
                    if objectives_yaml and not objectives_yaml.startswith("#"):
                        obj_data = yaml.safe_load(objectives_yaml)
                        if (
                            obj_data
                            and "physics" in obj_data
                            and "backend" in obj_data["physics"]
                        ):
                            backend = SimulatorBackendType(
                                obj_data["physics"]["backend"]
                            )
                except Exception:
                    logger.warning("failed_to_parse_backend_from_objectives")

                sim_res = await ctx.worker_client.simulate(
                    script_path=SCRIPT_FILE, backend=backend
                )
                if not sim_res.success:
                    from shared.simulation.schemas import ValidationResult

                    state.simulation_result = ValidationResult(
                        valid=False,
                        cost=0,
                        logs=[f"Physics simulation failed: {sim_res.message}"],
                        render_paths=[],
                        render_data=[],
                    )
                else:
                    # Download Renders
                    render_paths = (
                        sim_res.artifacts.get("render_paths", [])
                        if sim_res.artifacts
                        else []
                    )

                    async def _download(url_path):
                        url = f"{worker_url}/assets/{url_path.lstrip('/')}"
                        try:
                            # We still need httpx for direct asset download
                            async with httpx.AsyncClient() as http_client:
                                r = await http_client.get(
                                    url, headers={"X-Session-ID": session_id}
                                )
                                return r.content if r.status_code == 200 else None
                        except Exception:
                            return None

                    tasks = [_download(p) for p in render_paths]
                    results = await asyncio.gather(*tasks)
                    render_data = [r for r in results if r is not None]

                    from shared.simulation.schemas import ValidationResult

                    state.simulation_result = ValidationResult(
                        valid=True,
                        cost=0,
                        logs=["Validation passed."],
                        render_paths=render_paths,
                        render_data=render_data,
                    )
        except Exception as e:
            logger.error("integrated_validation_error", error=str(e))

    return state


class BenchmarkCOTSSearchSignature(dspy.Signature):
    """
    COTS Search node: Searches for components based on current needs for the benchmark.
    You must use the provided tools to search for components.
    When done, use SUBMIT to provide a summary of the components found.
    """

    prompt = dspy.InputField()
    search_summary = dspy.OutputField(desc="A summary of the components found")


async def cots_search_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    # Re-implementing to ensure consistency
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)

    # Setup DSPy Program
    from controller.agent.dspy_utils import WorkerInterpreter

    interpreter = WorkerInterpreter(
        worker_client=ctx.worker_client, session_id=session_id
    )

    tools = get_benchmark_tools(ctx.fs, session_id)
    tool_functions = {}
    for t in tools:
        func = getattr(t, "func", getattr(t, "_run", None))
        if func:
            tool_functions[t.name] = func

    program = dspy.CodeAct(
        BenchmarkCOTSSearchSignature,
        tools=list(tool_functions.values()),
        interpreter=interpreter,
    )

    try:
        with dspy.settings.context(lm=ctx.dspy_lm):
            logger.info("cots_search_dspy_invoke_start", session_id=session_id)
            prediction = program(prompt=state.session.prompt)
            logger.info("cots_search_dspy_invoke_complete", session_id=session_id)

        summary = getattr(prediction, "search_summary", "No summary provided.")
        state.messages.append(AIMessage(content=f"COTS Search summary: {summary}"))

    except Exception as e:
        logger.error("cots_search_dspy_failed", error=str(e))
    finally:
        interpreter.shutdown()

    return state


async def skills_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    # No changes needed
    return state


class BenchmarkReviewerSignature(dspy.Signature):
    """
    Agentic review of the generated benchmark.
    You must use the provided tools to inspect the workspace.
    You MUST use the 'write_review_file' tool to persist your review.
    When done, use SUBMIT to provide the final review result.
    """

    theme = dspy.InputField()
    prompt = dspy.InputField()
    benchmark_structure = dspy.InputField()
    objectives = dspy.InputField()
    simulation_logs = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Agentic review of the generated benchmark.
    Refactored to use DSPy CodeAct with remote worker execution.
    """
    session_id = str(state.session.session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    ctx = SharedNodeContext.create(worker_url=worker_url, session_id=session_id)
    logger.info("reviewer_node_start", round=state.review_round)

    # Read context files
    benchmark_structure = "# No benchmark_structure.md found."
    with suppress(Exception):
        if await ctx.worker_client.exists("benchmark_structure.md"):
            benchmark_structure = await ctx.worker_client.read_file(
                "benchmark_structure.md"
            )

    objectives = "# No objectives.yaml found."
    with suppress(Exception):
        if await ctx.worker_client.exists("objectives.yaml"):
            objectives = await ctx.worker_client.read_file("objectives.yaml")

    state.review_round = state.review_round + 1
    review_filename = f"reviews/review-round-{state.review_round}/review.md"

    # Setup Tools
    all_tools = get_benchmark_tools(ctx.fs, session_id)
    tool_functions = {}

    async def write_review_file(path: str, content: str) -> str:
        """Write the review to the review file."""
        p = path.lstrip("/")
        if p != review_filename.lstrip("/"):
            return f"Error: Unauthorized path. You must write to {review_filename}"
        success = await ctx.worker_client.write_file(path, content)
        return "Review written successfully." if success else "Error writing review."

    tool_functions["write_review_file"] = write_review_file

    for t in all_tools:
        if t.name not in ("write_file", "edit_file", "submit_for_review"):
            func = getattr(t, "func", getattr(t, "_run", None))
            if func:
                tool_functions[t.name] = func

    from controller.agent.dspy_utils import WorkerInterpreter

    interpreter = WorkerInterpreter(
        worker_client=ctx.worker_client, session_id=session_id
    )
    program = dspy.CodeAct(
        BenchmarkReviewerSignature.with_instructions(
            ctx.pm.render("benchmark_reviewer")
        ),
        tools=list(tool_functions.values()),
        interpreter=interpreter,
    )

    try:
        with dspy.settings.context(lm=ctx.dspy_lm):
            logger.info("reviewer_dspy_invoke_start", session_id=session_id)
            prediction = program(
                theme=state.plan.theme if state.plan else "Unknown",
                prompt=state.session.prompt,
                benchmark_structure=benchmark_structure,
                objectives=objectives,
                simulation_logs=str(
                    state.simulation_result.logs if state.simulation_result else []
                ),
            )
            logger.info("reviewer_dspy_invoke_complete", session_id=session_id)

        review = prediction.review
        state.review_feedback = f"{review.decision.value}: {review.reason}"
        if review.required_fixes:
            state.review_feedback += "\nFixes: " + ", ".join(review.required_fixes)

    except Exception as e:
        logger.error("reviewer_dspy_failed", error=str(e))
        state.review_feedback = f"Error executing reviewer: {e}"
    finally:
        interpreter.shutdown()

    return state
