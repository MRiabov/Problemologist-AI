import logging

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import CostWeightDeltaEvent, RunCommandToolEvent
from shared.type_checking import type_check

from contextlib import suppress
from ..config import settings
from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)


@type_check
class CoderNode:
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model=settings.llm_model, temperature=0)
        # T011 & T012: Initialize middleware/client
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.fs = RemoteFilesystemMiddleware(self.worker_client)

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the coder node logic."""
        # T010: Find next active item in TODO
        todo = state.todo
        current_step = self._get_next_step(todo)
        if not current_step:
            return state.model_copy(
                update={"journal": state.journal + "\nNo more steps in TODO."}
            )

        # T013: Write -> Run -> Fix loop
        max_retries = 3
        retry_count = 0
        last_error = ""
        journal_entry = f"\nStarting step: {current_step}"

        while retry_count < max_retries:
            # 1. Generate Python code
            prompt = self.pm.render(
                "engineer", current_step=current_step, error=last_error, plan=state.plan
            )
            response = await self.llm.ainvoke([HumanMessage(content=prompt)])
            code = self._extract_code(str(response.content))

            # 2. Execute code (T012)
            # Note: T012 mentions Temporal, but since WP06 is planned,
            # we use direct execution for now.
            try:
                # Record tool invocation
                await record_worker_events(
                    episode_id=state.session_id,
                    events=[RunCommandToolEvent(command=code)],
                )

                execution_result = await self.fs.run_command(code)

                stdout = execution_result.get("stdout", "")
                stderr = execution_result.get("stderr", "")
                exit_code = execution_result.get("exit_code", 0)
                events = execution_result.get("events", [])

                # Record events for observability
                if events:
                    await record_worker_events(
                        episode_id=state.session_id,
                        events=events,
                    )

                if exit_code == 0:
                    # T015: Validation Gate after successful execution
                    from worker.utils.file_validation import validate_node_output

                    # Read current files from worker to validate
                    # For coder, we check plan, todo, and objectives
                    all_files = {}
                    for f in ["plan.md", "todo.md", "objectives.yaml"]:
                        with suppress(Exception):
                            all_files[f] = await self.fs.read_file(f)

                    is_valid, validation_errors = validate_node_output(
                        "coder", all_files
                    )
                    if not is_valid:
                        error_msg = "Coder produced invalid output:\n" + "\n".join(
                            [f"- {e}" for e in validation_errors]
                        )
                        # We treat this as a failure that needs a retry/fix
                        last_error = error_msg
                        journal_entry += (
                            f"\nValidation failed after executive: {validation_errors}"
                        )
                        retry_count += 1
                        continue

                    journal_entry += f"\nSuccessfully executed step: {current_step}"
                    # Mark TODO as done (simple string replacement for prototype)
                    new_todo = todo.replace(
                        f"- [ ] {current_step}", f"- [x] {current_step}"
                    )

                    # Track best cost/weight from events
                    best_cost = state.best_cost
                    best_weight = state.best_weight_g
                    current_cost = None
                    current_weight = None

                    for event in events:
                        if hasattr(event, "price") and event.price is not None:
                            current_cost = (current_cost or 0) + event.price
                        if hasattr(event, "weight_g") and event.weight_g is not None:
                            current_weight = (current_weight or 0) + event.weight_g

                    if current_cost is not None:
                        if best_cost is None or current_cost < best_cost:
                            best_cost = current_cost
                        if best_weight is None or current_weight < best_weight:
                            best_weight = current_weight

                        # If this wasn't the best, log a delta (simplified heuristic)
                        if best_cost is not None and current_cost > best_cost:
                            await record_worker_events(
                                episode_id=state.session_id,
                                events=[
                                    CostWeightDeltaEvent(
                                        best_simulated_cost=best_cost,
                                        best_simulated_weight_g=best_weight or 0.0,
                                        final_cost=current_cost,
                                        final_weight_g=current_weight or 0.0,
                                        is_worse=True,
                                    )
                                ],
                            )

                    return state.model_copy(
                        update={
                            "todo": new_todo,
                            "journal": state.journal + journal_entry,
                            "current_step": current_step,
                            "best_cost": best_cost,
                            "best_weight_g": best_weight,
                            "turn_count": state.turn_count + 1,
                        }
                    )
                last_error = stderr or stdout or "Unknown error"
                journal_entry += (
                    f"\nExecution failed (Attempt {retry_count + 1}): {last_error}"
                )
                retry_count += 1
            except Exception as e:
                last_error = str(e)
                journal_entry += f"\nSystem error during execution: {last_error}"
                retry_count += 1

        journal_entry += f"\nFailed to complete step after {max_retries} retries."
        return state.model_copy(
            update={
                "journal": state.journal + journal_entry,
                "iteration": state.iteration + 1,
                "turn_count": state.turn_count + 1,
            }
        )

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list."""
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                return line.strip().replace("- [ ]", "").strip()
        return None

    def _extract_code(self, content: str) -> str:
        """Extract Python code from markdown block if present."""
        if "```python" in content:
            return content.split("```python")[1].split("```")[0].strip()
        if "```" in content:
            return content.split("```")[1].split("```")[0].strip()
        return content.strip()


# Factory function for LangGraph
@type_check
async def coder_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    node = CoderNode(worker_url=settings.spec_001_api_url, session_id=session_id)
    return await node(state)
