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
from controller.observability.tracing import record_worker_events, sync_asset
from controller.utils.git import GitManager
from shared.enums import SessionStatus
from shared.models.schemas import ReviewResult
from shared.observability.schemas import SkillEditEvent
from shared.simulation.schemas import (
    RandomizationStrategy,
    SimulatorBackendType,
)
from shared.type_checking import type_check
from shared.workers.schema import SimulationArtifacts

from .state import BenchmarkGeneratorState
from .tools import get_benchmark_planner_tools, get_benchmark_tools

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

    async def _write_fallback_planner_files(
        self, prompt: str, state: BenchmarkGeneratorState
    ) -> None:
        """Write a minimal valid benchmark planning bundle when ReAct planning fails."""
        theme = "sideways_ball_transfer"
        if "fluid" in prompt.lower():
            theme = "fluid_guidance"
        elif "electronics" in prompt.lower() or "circuit" in prompt.lower():
            theme = "motorized_transfer"

        plan_md = f"""# Benchmark Plan

## 1. Learning Objective
- Design a benchmark where an engineered mechanism moves a steel ball laterally to a goal area.
- Train lateral transport under gravity with explicit start/goal geometry.

## 2. Geometry
- Build zone: 1.0m x 1.0m x 1.0m.
- Start zone at one side of the workspace.
- Goal zone on the opposite side with no direct overlap with the start zone.

## 3. Objectives
- Success: the moved object center enters `goal_zone`.
- Failure: entering any `forbid_zones` or leaving `build_zone`.
- Randomization: apply small runtime jitter at spawn.

## 4. Randomization
- Runtime jitter on moved object: +/- 10mm on X/Y, 0mm on Z.
- Keep obstacle randomization conservative for first-pass solvability.

## 5. Cost Envelope
- max_unit_cost: 50.0 USD
- max_weight_g: 500.0 g

## 6. Theme
- {theme}
"""

        todo_md = """# TODO
- [ ] Define final benchmark geometry in script.py
- [ ] Validate geometry bounds and non-intersection
- [ ] Simulate with runtime jitter
- [ ] Confirm success/failure zones are correctly placed
"""

        objectives_yaml = """objectives:
  goal_zone:
    min: [850.0, 450.0, 0.0]
    max: [980.0, 550.0, 180.0]
  forbid_zones: []
  build_zone:
    min: [0.0, 0.0, 0.0]
    max: [1000.0, 1000.0, 1000.0]
  fluid_objectives: []
  stress_objectives: []
moved_object:
  label: "steel_ball"
  shape: "sphere"
  static_randomization:
    radius: [39.5, 40.5]
  start_position: [120.0, 500.0, 80.0]
  runtime_jitter: [10.0, 10.0, 0.0]
constraints:
  max_unit_cost: 50.0
  max_weight_g: 500.0
randomization:
  static_variation_id: "fallback-v1"
  runtime_jitter_enabled: true
physics:
  backend: GENESIS
  fem_enabled: false
  compute_target: auto
"""

        await self.ctx.worker_client.write_file("plan.md", plan_md, overwrite=True)
        await self.ctx.worker_client.write_file("todo.md", todo_md, overwrite=True)
        await self.ctx.worker_client.write_file(
            "objectives.yaml", objectives_yaml, overwrite=True
        )

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
            "journal": state.journal,
            "review_feedback": (
                state.review_feedback
                if state.session.status == SessionStatus.REJECTED
                else "No feedback yet."
            ),
        }

        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            BenchmarkPlannerSignature,
            state,
            inputs,
            get_benchmark_planner_tools,
            ["plan.md", "todo.md", "objectives.yaml"],
            "benchmark_planner",
            max_retries=1,
        )

        if not prediction:
            await self._write_fallback_planner_files(state.session.prompt, state)
            state.plan = RandomizationStrategy(
                theme="fallback", reasoning="Planner fallback after model failure"
            )
            state.journal += f"\n[Planner] Fallback used: {journal_entry}"
            return state

        state.plan = prediction.plan
        state.journal += (
            f"\n[Planner] {getattr(prediction, 'reasoning', '')}\n{journal_entry}"
        )
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
        worker_light_url=worker_light_url,
        session_id=session_id,
        episode_id=state.episode_id,
        agent_role="benchmark_planner",
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


@type_check
class BenchmarkCoderNode(BaseNode):
    """Refactored Benchmark Coder using BaseNode."""

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        session_id = str(state.session.session_id)
        logger.info("coder_node_start", session_id=session_id)

        # Context construction
        validation_logs = "\n".join(state.session.validation_logs)
        if state.simulation_result and not state.simulation_result.valid:
            validation_logs += "\n" + "\n".join(state.simulation_result.logs)

        objectives_yaml = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists(OBJECTIVES_FILE):
                objectives_yaml = await self.ctx.worker_client.read_file(
                    OBJECTIVES_FILE
                )

        inputs = {
            "prompt": state.session.prompt,
            "plan": state.plan.model_dump_json() if state.plan else "None",
            "objectives_yaml": objectives_yaml,
            "review_feedback": (
                state.review_feedback
                if state.session.status == SessionStatus.REJECTED
                else "No feedback provided."
            ),
            "validation_logs": validation_logs,
        }

        # Node uses ReAct for tool usage
        prediction, artifacts, journal_entry = await self._run_program(
            dspy.ReAct,
            BenchmarkCoderSignature,
            state,
            inputs,
            get_benchmark_tools,
            [SCRIPT_FILE, "plan.md", "todo.md", "objectives.yaml"],
            "benchmark_coder",
        )

        if not prediction and not artifacts:
            state.journal += f"\n[Coder] Failed: {journal_entry}"
            return state

        new_journal = (
            getattr(prediction, "journal", "No journal provided.")
            if prediction
            else "Validation failed, but some artifacts were created."
        )
        state.journal += f"\n[Coder] {new_journal}"
        state.messages.append(AIMessage(content=f"Work summary: {new_journal}"))

        # Retrieve script for further validation/simulation
        if SCRIPT_FILE in artifacts:
            state.current_script = artifacts[SCRIPT_FILE]
        else:
            with suppress(Exception):
                state.current_script = await self.ctx.worker_client.read_file(
                    SCRIPT_FILE
                )

        # Run Verification (Geometric + Physics)
        script = state.current_script
        if script:
            logger.info("running_integrated_validation", session_id=session_id)
            try:
                val_res = await self.ctx.worker_client.validate(script_path=SCRIPT_FILE)
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
                        pass

                    sim_res = await self.ctx.worker_client.simulate(
                        script_path=SCRIPT_FILE, backend=backend
                    )

                    # Update MJCF content from simulation results
                    if sim_res.artifacts:
                        if isinstance(sim_res.artifacts, SimulationArtifacts):
                            if sim_res.artifacts.mjcf_content:
                                state.mjcf_content = sim_res.artifacts.mjcf_content
                        elif isinstance(sim_res.artifacts, dict):
                            state.mjcf_content = sim_res.artifacts.get(
                                "mjcf_content", ""
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
                        render_paths = []
                        if sim_res.artifacts:
                            if isinstance(sim_res.artifacts, SimulationArtifacts):
                                render_paths = sim_res.artifacts.render_paths
                            elif isinstance(sim_res.artifacts, dict):
                                render_paths = sim_res.artifacts.get("render_paths", [])

                        async def _download(url_path):
                            from controller.config.settings import (
                                settings as global_settings,
                            )

                            worker_light_url = global_settings.worker_light_url
                            url = f"{worker_light_url}/assets/{url_path.lstrip('/')}"
                            try:
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
                state.session.status = SessionStatus.FAILED

        return state


@type_check
async def coder_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url,
        session_id=session_id,
        episode_id=state.episode_id,
        agent_role="benchmark_cad_coder",
    )
    node = BenchmarkCoderNode(context=ctx)
    return await node(state)


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
            dspy.ReAct,
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
        worker_light_url=worker_light_url,
        session_id=session_id,
        episode_id=state.episode_id,
        agent_role="benchmark_cad_coder",
    )
    node = BenchmarkCOTSSearchNode(context=ctx)
    return await node(state)


class BenchmarkSkillsSignature(dspy.Signature):
    """
    Skills node: Analyzes the benchmark generation journal to suggest new skills.
    You must use the provided tools to analyze the work and save new skills.
    If you identify a valuable benchmark pattern or physics setup, use the `save_suggested_skill` tool to record it.
    When done, use SUBMIT to provide a summary of your work.
    """

    prompt = dspy.InputField()
    journal = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the skills identified and saved")


@type_check
class BenchmarkSkillsNode(BaseNode):
    """
    Benchmark Skills node: Analyzes the journal to suggest new skills for benchmark generation.
    """

    def __init__(
        self,
        context: SharedNodeContext,
        suggested_skills_dir: str = "suggested_skills/benchmark",
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
        """Sync changes with git."""
        await self.git.sync_changes(commit_message, lm=self.ctx.dspy_lm)

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        """Execute the skills learner logic."""

        async def save_suggested_skill(title: str, content: str) -> str:
            """
            Saves a new suggested skill to the repository.
            """
            clean_title = title.strip().lower().replace(" ", "_").replace(".md", "")
            file_path = self.suggested_skills_dir / f"{clean_title}.md"

            # Safety toggle for deletions (from T030)
            if file_path.exists():
                import difflib

                old_lines = file_path.read_text().splitlines()
                new_lines = content.splitlines()
                diff = list(difflib.unified_diff(old_lines, new_lines, n=0))
                deletions = sum(
                    1
                    for line in diff
                    if line.startswith("-") and not line.startswith("---")
                )
                if deletions > 15:
                    logger.warning(
                        "skill_update_blocked_too_many_deletions", deletions=deletions
                    )
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
                    return f"Error: Update blocked. You deleted {deletions} lines."

            try:
                with file_path.open("w") as f:
                    f.write(content)

                # Sync to Database as Asset
                try:
                    episode_uuid = uuid.UUID(self.ctx.session_id)
                    await sync_asset(
                        episode_uuid,
                        f"suggested_skills/benchmark/{clean_title}.md",
                        content,
                    )
                except Exception as e:
                    logger.warning(f"Failed to sync skill asset: {e}")

                # Sync to Git
                await self._sync_git(f"Add benchmark skill: {clean_title}")

                return f"Benchmark skill '{clean_title}' saved and synced successfully."
            except Exception as e:
                return f"Error saving skill: {e}"

        def get_skills_tools(_fs, _session_id):
            # BaseNode._run_program passes (fs, session_id) to the tool_factory
            # We wrap the local save_suggested_skill into the toolset
            tools = get_benchmark_tools(self.ctx.fs, self.ctx.session_id)
            tools.append(save_suggested_skill)
            return tools

        inputs = {
            "prompt": state.session.prompt,
            "journal": state.journal,
        }

        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            BenchmarkSkillsSignature,
            state,
            inputs,
            get_skills_tools,
            [],
            "benchmark_skill_learner",
        )

        if not prediction:
            state.journal += f"\n[Skills] Failed: {journal_entry}"
            return state

        summary = getattr(prediction, "summary", "No summary provided.")
        state.journal += f"\n[Skills] {summary}" + journal_entry
        state.messages.append(AIMessage(content=f"Benchmark skills update: {summary}"))
        return state


@type_check
async def skills_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url,
        session_id=session_id,
        episode_id=state.episode_id,
        agent_role="benchmark_planner",
    )
    node = BenchmarkSkillsNode(context=ctx)
    return await node(state)


class SummarizerSignature(dspy.Signature):
    """
    Summarizer node: Compresses the journal to stay within token limits.
    Provide a concise summary of the key decisions, attempts, and outcomes
    recorded in the journal.
    Maintain critical technical details while reducing verbosity.
    """

    journal = dspy.InputField()
    summarized_journal = dspy.OutputField(desc="A concise summary of the journal")


@type_check
class BenchmarkSummarizerNode(BaseNode):
    """
    Summarizer node: Compresses the journal when it exceeds length limits.
    """

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        if not state.journal or len(state.journal) < 5000:
            return state

        logger.info(
            "summarizing_benchmark_journal",
            journal_length=len(state.journal),
            session_id=str(state.session.session_id),
        )

        inputs = {"journal": state.journal}
        program = dspy.ReAct(SummarizerSignature, tools=[])

        with dspy.settings.context(lm=self.ctx.dspy_lm):
            prediction = await asyncio.to_thread(program, **inputs)

        summarized = getattr(prediction, "summarized_journal", state.journal)

        logger.info(
            "benchmark_journal_summarized",
            old_length=len(state.journal),
            new_length=len(summarized),
        )

        state.journal = f"[Summarized Journal]\n{summarized}"
        return state


@type_check
async def summarizer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    session_id = str(state.session.session_id)
    from controller.config.settings import settings as global_settings

    worker_light_url = global_settings.worker_light_url
    ctx = SharedNodeContext.create(
        worker_light_url=worker_light_url,
        session_id=session_id,
        episode_id=state.episode_id,
        agent_role="benchmark_planner",
    )
    node = BenchmarkSummarizerNode(context=ctx)
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
    plan_md = dspy.InputField()
    objectives = dspy.InputField()
    simulation_logs = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class BenchmarkReviewerNode(BaseNode):
    """Refactored Benchmark Reviewer using BaseNode."""

    async def __call__(self, state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
        logger.info("reviewer_node_start", round=state.review_round)

        # Read context files
        plan_md = "# No plan.md found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("plan.md"):
                plan_md = await self.ctx.worker_client.read_file("plan.md")

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
            final_tools = []
            for t in tools:
                name = getattr(t, "name", getattr(t, "__name__", None))
                if name not in ("write_file", "edit_file", "submit_for_review"):
                    final_tools.append(t)

            final_tools.append(write_review_file)
            return final_tools

        inputs = {
            "theme": state.plan.theme if state.plan else "Unknown",
            "prompt": state.session.prompt,
            "plan_md": plan_md,
            "objectives": objectives,
            "simulation_logs": str(
                state.simulation_result.logs if state.simulation_result else []
            ),
        }

        try:
            prediction, _, journal_entry = await self._run_program(
                dspy.ReAct,
                BenchmarkReviewerSignature,
                state,
                inputs,
                get_reviewer_tools,
                [],
                "benchmark_reviewer",
            )

            if not prediction:
                state.review_feedback = "Error: Reviewer failed to complete."
                state.journal += f"\n[Reviewer] Failed: {journal_entry}"
                return state

            review = prediction.review
            state.review_decision = review.decision
            state.review_feedback = f"{review.decision.value}: {review.reason}"
            state.journal += f"\n[Reviewer] {state.review_feedback}\n{journal_entry}"
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
        worker_light_url=worker_light_url,
        session_id=session_id,
        episode_id=state.episode_id,
        agent_role="engineering_reviewer",
    )
    node = BenchmarkReviewerNode(context=ctx)
    return await node(state)
