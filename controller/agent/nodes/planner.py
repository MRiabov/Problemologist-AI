import structlog
from pathlib import Path
from contextlib import suppress

from typing import cast
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import get_langfuse_callback
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

logger = structlog.get_logger(__name__)


@type_check
class PlannerNode:
    """
    Planner node: Analyzes the task and creates plan.md and todo.md using tools.
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
        """Execute the planner node logic."""
        # T006: Read skills (local controller FS is fine for skills)
        skills_dir = Path(".agent/skills")
        skills = []
        if skills_dir.exists():
            skills = [d.name for d in skills_dir.iterdir() if d.is_dir()]

        skills_context = "\n".join([f"- {s}" for s in skills])

        # WP04: Extract steerability context from last message if present
        steer_context = ""
        if state.messages:
            last_msg = state.messages[-1]
            steer_data = (
                last_msg.additional_kwargs.get("steerability")
                if hasattr(last_msg, "additional_kwargs")
                else None
            )

            if steer_data:
                lines = []
                if steer_data.get("selections"):
                    lines.append("Geometric Selections:")
                    for sel in steer_data["selections"]:
                        center_str = ", ".join([f"{v:.2f}" for v in sel["center"]])
                        lines.append(
                            f"  - Level: {sel['level']}, ID: {sel['target_id']}, Center: ({center_str})"
                        )

                if steer_data.get("code_references"):
                    lines.append("Code References:")
                    for ref in steer_data["code_references"]:
                        lines.append(
                            f"  - File: {ref['file_path']}, Lines: {ref['start_line']}-{ref['end_line']}"
                        )

                if steer_data.get("mentions"):
                    lines.append(f"Mentions: {', '.join(steer_data['mentions'])}")

                steer_context = "\n".join(lines)

        # Observability
        langfuse_callback = get_langfuse_callback(
            name="planner", session_id=state.session_id
        )
        db_callback = DatabaseCallbackHandler(
            episode_id=state.session_id, langfuse_callback=langfuse_callback
        )
        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)

        # T005: Invoke LLM
        prompt_text = self.pm.render(
            "architect",
            task=state.task,
            skills=skills_context,
            steer_context=steer_context,
        )
        # Instruct agent to use tools to write files
        prompt_text += "\n\nIMPORTANT: You must use the 'write_file' tool to create 'plan.md' and 'todo.md' directly. Do not just output the content in the chat."

        messages = [SystemMessage(content=prompt_text)]

        max_retries = 3
        retry_count = 0
        journal_entry = "\n[Planner] Starting planning phase."

        while retry_count < max_retries:
            try:
                # Invoke agent
                logger.info("planner_agent_invoke_start", session_id=state.session_id)
                result = await self.agent.ainvoke(
                    {"messages": messages}, config={"callbacks": callbacks}
                )
                logger.info(
                    "planner_agent_invoke_complete", session_id=state.session_id
                )
                messages = result["messages"]

                # Validation Gate
                from worker.utils.file_validation import validate_node_output

                # Read artifacts to validate
                artifacts = {}
                for f in ["plan.md", "todo.md"]:
                    with suppress(Exception):
                        content = await self.fs.read_file(f)
                        artifacts[f] = content

                is_valid, validation_errors = validate_node_output("planner", artifacts)

                if not is_valid:
                    error_msg = f"Planner output validation failed: {validation_errors}"
                    journal_entry += f"\nValidation failed (Attempt {retry_count + 1}): {validation_errors}"
                    messages.append(HumanMessage(content=error_msg))

                    if retry_count == max_retries - 1:
                        # Last attempt failed
                        return state.model_copy(
                            update={
                                "status": AgentStatus.PLAN_REJECTED,
                                "feedback": error_msg,
                                "journal": state.journal + journal_entry,
                                "messages": messages,
                            }
                        )

                    retry_count += 1
                    continue

                # Success
                # Emit SubmissionValidationEvent
                await record_worker_events(
                    episode_id=state.session_id,
                    events=[
                        SubmissionValidationEvent(
                            artifacts_present=list(artifacts.keys()),
                            verification_passed=True,
                            reasoning_trace_quality=1.0,  # Placeholder
                            errors=[],
                        )
                    ],
                )

                return state.model_copy(
                    update={
                        "plan": artifacts.get("plan.md", ""),
                        "todo": artifacts.get("todo.md", ""),
                        "status": AgentStatus.EXECUTING,
                        "journal": state.journal + journal_entry,
                        "messages": messages,
                    }
                )

            except Exception as e:
                journal_entry += f"\nSystem error during planning: {e}"
                messages.append(HumanMessage(content=f"System error: {e}"))
                retry_count += 1

        return state.model_copy(
            update={
                "status": AgentStatus.FAILED,
                "journal": state.journal + journal_entry,
                "messages": messages,
            }
        )


# Factory function for LangGraph
@type_check
async def planner_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    node = PlannerNode(worker_url=settings.spec_001_api_url, session_id=session_id)
    return await node(state)
