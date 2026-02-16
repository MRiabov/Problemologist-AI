import logging
import structlog
from contextlib import suppress

from typing import cast
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.observability.schemas import ElecAgentHandoverEvent
from controller.observability.tracing import record_worker_events
from shared.type_checking import type_check

logger = structlog.get_logger(__name__)


@type_check
class ElectronicsEngineerNode:
    """
    Electronics Engineer node: Designs circuits and routes wires.
    Refactored to use LangGraph's prebuilt ReAct agent.
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

        # Initialize tools and agent
        self.tools = get_engineer_tools(self.fs, session_id)
        self.agent = create_react_agent(self.llm, self.tools)

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
                # No specific requirements, check assembly_definition if needed
                # For now, simple check as per original code
                pass
        except Exception:
            pass

        # 2. Get current assembly context
        assembly_context = "No assembly context available."
        try:
            # Check assembly_definition.yaml (Assembly Definition)
            with suppress(Exception):
                assembly_context = await self.fs.read_file("assembly_definition.yaml")
        except Exception:
            pass

        # 3. Generate electronics implementation code
        prompt = self.pm.render(
            "electronics_engineer",
            current_step="Design circuit and route wires",
            plan=state.plan,
            assembly_context=assembly_context,
        )

        messages = [SystemMessage(content=prompt)]

        max_retries = 3
        retry_count = 0
        journal_entry = "\n[Electronics Engineer] Starting circuit design and routing."

        while retry_count < max_retries:
            try:
                # Invoke agent
                result = await self.agent.ainvoke({"messages": messages})
                messages = result["messages"]

                # 4. Validate output
                from worker.utils.file_validation import validate_node_output

                all_files = {}
                for f in ["plan.md", "todo.md"]:
                    with suppress(Exception):
                        all_files[f] = await self.fs.read_file(f)

                is_valid, validation_errors = validate_node_output(
                    "electronics_engineer", all_files
                )

                if not is_valid:
                    error_msg = f"Circuit implementation produced invalid output: {validation_errors}"
                    journal_entry += f"\nValidation failed (Attempt {retry_count + 1}): {validation_errors}"
                    logger.warning(
                        "electronics_engineer_validation_failed",
                        errors=validation_errors,
                    )
                    messages.append(HumanMessage(content=error_msg))
                    retry_count += 1
                    continue

                # Success
                journal_entry += "\nCircuit and wiring implementation successful."
                return state.model_copy(
                    update={
                        "journal": state.journal + journal_entry,
                        "turn_count": state.turn_count + 1,
                        "messages": messages,
                    }
                )

            except Exception as e:
                journal_entry += (
                    f"\nSystem error during electronics implementation: {e}"
                )
                logger.error("electronics_engineer_system_error", error=str(e))
                messages.append(HumanMessage(content=f"System error: {e}"))
                retry_count += 1

        journal_entry += f"\nFailed to complete step after {max_retries} retries."
        return state.model_copy(
            update={
                "journal": state.journal + journal_entry,
                "turn_count": state.turn_count + 1,
                "messages": messages,
            }
        )


# Factory function for LangGraph
@type_check
async def electronics_engineer_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    node = ElectronicsEngineerNode(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    return await node(state)
