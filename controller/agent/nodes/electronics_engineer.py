import structlog
from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ElecAgentHandoverEvent, RunCommandToolEvent
from shared.type_checking import type_check

from contextlib import suppress
from ..config import settings
from ..prompt_manager import PromptManager
from ..state import AgentState

logger = structlog.get_logger(__name__)


@type_check
class ElectronicsEngineerNode:
    """
    Electronics Engineer node: Designs circuits and routes wires.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model=settings.llm_model, temperature=0)
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.fs = RemoteFilesystemMiddleware(self.worker_client)

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics engineer node logic."""
        # 1. Check if electronics are needed
        try:
            # Emit handover event
            await record_worker_events(
                episode_id=state.session_id,
                events=[
                    ElecAgentHandoverEvent(
                        from_agent="Mechanical Engineer",
                        to_agent="Electronics Engineer",
                        iteration_count=state.iteration,
                    )
                ],
            )

            objectives_content = await self.fs.read_file("objectives.yaml")
            if "electronics_requirements" not in objectives_content:
                # No specific requirements, but we might still have motors to wire
                # if the planner put them in assembly_definition.yaml.
                pass
        except Exception as e:
            logger.warning("electronics_engineer_check_requirements_failed", error=str(e))

        # 2. Get current assembly context
        assembly_context = "No assembly context available."
        try:
            # Check assembly_definition.yaml (Assembly Definition)
            assembly_context = await self.fs.read_file("assembly_definition.yaml")
        except Exception as e:
            logger.warning("electronics_engineer_read_assembly_context_failed", error=str(e))

        # 3. Generate electronics implementation code with retry loop
        max_retries = 3
        retry_count = 0
        last_error = ""
        journal_entry = "\n[Electronics Engineer] Starting circuit design and routing."

        while retry_count < max_retries:
            try:
                prompt = self.pm.render(
                    "electronics_engineer",
                    current_step="Design circuit and route wires",
                    plan=state.plan,
                    assembly_context=assembly_context,
                    error=last_error,
                )
                response = await self.llm.ainvoke([HumanMessage(content=prompt)])
                code = self._extract_code(str(response.content))

                if not code:
                    # If LLM refuses to generate code (e.g., deemed unnecessary)
                    if retry_count > 0:
                        journal_entry += "\nAborted: LLM provided no code during retry."
                        return state.model_copy(update={"journal": state.journal + journal_entry})
                    return state

                # 4. Execute code
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

                    all_files = {}
                    for f in ["plan.md", "todo.md"]:
                        with suppress(Exception):
                            all_files[f] = await self.fs.read_file(f)

                    is_valid, validation_errors = validate_node_output(
                        "electronics_engineer", all_files
                    )
                    if is_valid:
                        journal_entry += "\nCircuit and wiring implementation successful."
                        # Success!
                        return state.model_copy(
                            update={
                                "journal": state.journal + journal_entry,
                                "turn_count": state.turn_count + 1,
                            }
                        )
                    else:
                        error_msg = f"Validation failed: {validation_errors}"
                        journal_entry += f"\nCircuit implementation produced invalid output: {error_msg}"
                        logger.warning(
                            "electronics_engineer_validation_failed",
                            errors=validation_errors,
                        )
                        last_error = error_msg
                        retry_count += 1
                else:
                    error_msg = stderr or stdout
                    journal_entry += f"\nCircuit implementation failed (Attempt {retry_count + 1}): {error_msg}"
                    logger.warning("electronics_engineer_failed", error=error_msg)
                    last_error = error_msg
                    retry_count += 1

            except Exception as e:
                error_msg = str(e)
                journal_entry += f"\nSystem error during electronics implementation: {error_msg}"
                logger.error("electronics_engineer_system_error", error=error_msg)
                last_error = error_msg
                retry_count += 1

        # If we exhausted retries
        journal_entry += f"\nFailed to complete electronics implementation after {max_retries} retries."
        return state.model_copy(
            update={
                "journal": state.journal + journal_entry,
                "turn_count": state.turn_count + 1,
            }
        )

    def _extract_code(self, content: str) -> str:
        """Extract Python code from markdown block if present."""
        if "```python" in content:
            return content.split("```python")[1].split("```")[0].strip()
        if "```" in content:
            return content.split("```")[1].split("```")[0].strip()
        return content.strip()


# Factory function for LangGraph
@type_check
async def electronics_engineer_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    node = ElectronicsEngineerNode(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    return await node(state)
