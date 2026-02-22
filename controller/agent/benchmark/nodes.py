import asyncio
import os
import re
import uuid
from contextlib import suppress
from pathlib import Path

import dspy
import httpx
import structlog
import yaml
from langchain_core.messages import AIMessage, HumanMessage

from controller.agent.nodes.base import BaseNode, SharedNodeContext
from controller.utils.git import GitManager
from controller.observability.tracing import record_worker_events, sync_asset
from shared.models.schemas import ReviewResult
from shared.enums import SessionStatus
from shared.observability.schemas import SkillEditEvent
from shared.simulation.schemas import (
    RandomizationStrategy,
    SimulatorBackendType,
)
from shared.type_checking import type_check

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
    reasoning = dspy.OutputField()
    plan: RandomizationStrategy = dspy.OutputField()


@type_check
class BenchmarkPlannerNode(BaseNode):
    """Refactored Benchmark Planner using BaseNode for prompt injection."""

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        # Init Git
        await self.ctx.worker_client.git_init()

        # Custom Objectives Logic
        custom_objectives = state.session.custom_objectives
        if custom_objectives:
            logger.info(
                "planner_updating_objectives", session_id=state.session.session_id
            )
            if await self.ctx.worker_client.exists(OBJECTIVES_FILE):
                try:
                    obj_content = await self.ctx.worker_client.read_file(
                        OBJECTIVES_FILE
                    )
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
                        obj_data["constraints"]["max_weight"] = (
                            custom_objectives.max_weight
                        )
                    if custom_objectives.target_quantity is not None:
                        obj_data["constraints"]["target_quantity"] = (
                            custom_objectives.target_quantity
                        )

                    new_content = yaml.dump(obj_data, sort_keys=False)
                    await self.ctx.worker_client.write_file(
                        OBJECTIVES_FILE, new_content
                    )
                    logger.info(
                        "planner_objectives_updated",
                        session_id=state.session.session_id,
                    )
                except Exception as e:
                    logger.warning("planner_objectives_update_failed", error=str(e))

        inputs = {
            "prompt": state.session.prompt,
            "history": str(state.messages or []),
            "review_feedback": (
                state.review_feedback
                if state.session.status == SessionStatus.REJECTED
                else "No feedback yet."
            ),
        }

        prediction, _, _ = await self._run_program(
            dspy.CodeAct,
            BenchmarkPlannerSignature,
            state,
            inputs,
            get_benchmark_tools,
            [],
            "benchmark_planner",
        )

        if not prediction:
            state.plan = RandomizationStrategy(
                theme="error", reasoning="Failed to plan"
            )
            return state

        state.plan = prediction.plan
        state.messages.append(
            HumanMessage(content=f"Generated plan: {state.plan.theme}")
        )
        return state


@type_check
async def planner_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url, session_id=session_id
    )
    node = BenchmarkPlannerNode(context=ctx)
    return await node(state)


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
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url

    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url, session_id=session_id
    )
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

    # WP08: Set node_type on mock LM if available for explicit lookup
    if hasattr(ctx.dspy_lm, "node_type"):
        ctx.dspy_lm.node_type = "benchmark_coder"

    # Invoke DSPy Program
    try:
        with dspy.settings.context(lm=ctx.dspy_lm):
            logger.info("coder_dspy_invoke_start", session_id=session_id)
            prediction = await asyncio.wait_for(
                asyncio.to_thread(
                    program,
                    prompt=state.session.prompt,
                    plan=state.plan.model_dump_json() if state.plan else "None",
                    objectives_yaml=objectives_yaml,
                    review_feedback=(
                        state.review_feedback
                        if state.session.status == SessionStatus.REJECTED
                        else "No feedback provided."
                    ),
                    validation_logs=validation_logs,
                ),
                timeout=300.0,
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
        from worker_heavy.utils.file_validation import validate_node_output

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
                        url = f"{worker_light_url}/assets/{url_path.lstrip('/')}"
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
            # Set a failure status and sleep to prevent infinite tight loop
            state.session.status = SessionStatus.FAILED
            await asyncio.sleep(2)

    return state


class BenchmarkCOTSSearchSignature(dspy.Signature):
    """
    COTS Search node: Searches for components based on current needs for the benchmark.
    You must use the provided tools to search for components.
    When done, use SUBMIT to provide a summary of the components found.
    """

    prompt = dspy.InputField()
    search_summary = dspy.OutputField(desc="A summary of the components found")


@type_check
class BenchmarkCOTSSearchNode(BaseNode):
    """Refactored Benchmark COTS Search using BaseNode."""

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        inputs = {"prompt": state.session.prompt}

        prediction, _, _ = await self._run_program(
            dspy.CodeAct,
            BenchmarkCOTSSearchSignature,
            state,
            inputs,
            get_benchmark_tools,
            [],
            "cots_search",
        )

        summary = getattr(prediction, "search_summary", "No summary provided.")
        state.messages.append(AIMessage(content=f"COTS Search summary: {summary}"))
        return state


@type_check
async def cots_search_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url, session_id=session_id
    )
    node = BenchmarkCOTSSearchNode(context=ctx)
    return await node(state)


class BenchmarkSkillsSignature(dspy.Signature):
    """
    Skills node: Analyzes the journal to suggest new skills.
    You must use the provided tools to analyze the work and save new skills.
    If you identify a valuable skill or pattern, use the `save_suggested_skill` tool to record it.
    When done, use SUBMIT to provide a summary of your work.
    """

    task = dspy.InputField()
    journal = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the skills identified and saved")


@type_check
class BenchmarkSkillsNode(BaseNode):
    """
    Skills node: Analyzes the journal to suggest new skills.
    Refactored to use DSPy ReAct with remote worker execution.
    """

    def __init__(
        self, context: SharedNodeContext, suggested_skills_dir: str = "suggested_skills"
    ):
        super().__init__(context)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)
        self.git = GitManager(
            repo_path=self.suggested_skills_dir,
            repo_url=os.getenv("GIT_REPO_URL"),
            pat=os.getenv("GIT_PAT"),
        )
        self.git.ensure_repo()

    async def _sync_git(self, commit_message: str):
        """Sync changes with git via GitManager."""
        await self.git.sync_changes(commit_message, lm=self.ctx.dspy_lm)

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        """Execute the sidecar node logic using DSPy ReAct."""

        async def save_suggested_skill(title: str, content: str) -> str:
            """
            Saves a new suggested skill to the repository.
            Args:
                title: A concise title for the skill (e.g., 'motor_initialization').
                content: The markdown content describing the skill.
            """
            clean_title = title.strip().lower().replace(" ", "_").replace(".md", "")
            file_path = self.suggested_skills_dir / f"{clean_title}.md"

            # T030: Safety toggle for deletions
            if file_path.exists():
                import difflib

                old_lines = file_path.read_text().splitlines()
                new_lines = content.splitlines()
                diff = list(difflib.unified_diff(old_lines, new_lines, n=0))
                # Count lines starting with '-' but not '---'
                deletions = sum(
                    1
                    for line in diff
                    if line.startswith("-") and not line.startswith("---")
                )
                if deletions > 15:
                    logger.warning(
                        "skill_update_blocked_too_many_deletions", deletions=deletions
                    )
                    # Emit safety event
                    await record_worker_events(
                        episode_id=self.ctx.session_id,
                        events=[
                            SkillEditEvent(
                                skill_name=clean_title,
                                action="update_blocked",
                                lines_changed=deletions,
                            )
                        ],
                    )
                    return f"Error: Update blocked. You deleted {deletions} lines, but the limit is 15 lines of deletion to prevent skill loss."

            try:
                with file_path.open("w") as f:
                    f.write(content)

                # Sync to Database as Asset
                try:
                    episode_uuid = uuid.UUID(self.ctx.session_id)
                    await sync_asset(
                        episode_uuid, f"suggested_skills/{clean_title}.md", content
                    )
                except Exception as e:
                    logger.warning(f"Failed to sync skill asset: {e}")

                # Sync to Git
                await self._sync_git(f"Add skill: {clean_title}")

                return f"Skill '{clean_title}' saved and synced successfully."
            except Exception as e:
                return f"Error saving skill: {e}"

        def get_skills_tools(fs, session_id):
            tools = get_benchmark_tools(fs, session_id)
            # Add specialized tool
            tools.append(save_suggested_skill)
            return tools

        # Construct synthetic journal from messages and validation logs
        journal_content = "\n".join([str(m.content) for m in state.messages])
        if state.session.validation_logs:
             journal_content += "\n\nValidation Logs:\n" + "\n".join(state.session.validation_logs)

        inputs = {
            "task": state.session.prompt,
            "journal": journal_content,
        }

        validate_files = []

        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            BenchmarkSkillsSignature,
            state,
            inputs,
            get_skills_tools,
            validate_files,
            "skill_learner",
        )

        if not prediction:
            # We don't fail the session if skills fail, just log it
            logger.warning("benchmark_skills_node_failed_no_prediction")
            return state

        summary = getattr(prediction, "summary", "No summary provided.")
        state.messages.append(AIMessage(content=f"Skills update: {summary}"))

        return state


async def skills_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url, session_id=session_id
    )
    node = BenchmarkSkillsNode(context=ctx)
    return await node(state)


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


@type_check
class BenchmarkReviewerNode(BaseNode):
    """Refactored Benchmark Reviewer using BaseNode."""

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        logger.info("reviewer_node_start", round=state.review_round)

        # Read context files
        benchmark_structure = "# No benchmark_structure.md found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("benchmark_structure.md"):
                benchmark_structure = await self.ctx.worker_client.read_file(
                    "benchmark_structure.md"
                )

        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        state.review_round = state.review_round + 1
        review_filename = f"reviews/review-round-{state.review_round}/review.md"

        # Specialized local tool
        async def write_review_file(path: str, content: str) -> str:
            """Write the review to the review file."""
            p = path.lstrip("/")
            if p != review_filename.lstrip("/"):
                return f"Error: Unauthorized path. You must write to {review_filename}"
            success = await self.ctx.worker_client.write_file(path, content)
            return (
                "Review written successfully." if success else "Error writing review."
            )

        def get_reviewer_tools(fs, session_id):
            tools = get_benchmark_tools(fs, session_id)
            # Filter tools and add specialized one
            final_tools = [
                t
                for t in tools
                if t.name not in ("write_file", "edit_file", "submit_for_review")
            ]
            final_tools.append(write_review_file)
            return final_tools

        inputs = {
            "theme": state.plan.theme if state.plan else "Unknown",
            "prompt": state.session.prompt,
            "benchmark_structure": benchmark_structure,
            "objectives": objectives,
            "simulation_logs": str(
                state.simulation_result.logs if state.simulation_result else []
            ),
        }

        try:
            prediction, _, _ = await self._run_program(
                dspy.CodeAct,
                BenchmarkReviewerSignature,
                state,
                inputs,
                get_reviewer_tools,
                [],
                "benchmark_reviewer",
            )

            if not prediction:
                state.review_feedback = "Error: Reviewer failed to complete."
                return state

            review = prediction.review
            state.review_decision = review.decision
            state.review_feedback = f"{review.decision.value}: {review.reason}"
            if review.required_fixes:
                state.review_feedback += "\nFixes: " + ", ".join(review.required_fixes)
        except Exception as e:
            logger.error("benchmark_reviewer_node_failed", error=str(e))
            state.review_feedback = "Rejected: Internal error"
            await asyncio.sleep(2)

        return state


@type_check
async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url, session_id=session_id
    )
    node = BenchmarkReviewerNode(context=ctx)
    return await node(state)
