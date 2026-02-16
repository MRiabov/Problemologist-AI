import logging
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
from shared.type_checking import type_check

logger = logging.getLogger(__name__)


@type_check
class CoderNode:
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
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
        """Execute the coder node logic."""
        # T010: Find next active item in TODO
        todo = state.todo
        current_step = self._get_next_step(todo)
        if not current_step:
            return state.model_copy(
                update={"journal": state.journal + "\nNo more steps in TODO."}
            )

        max_retries = 3
        retry_count = 0
        journal_entry = f"\nStarting step: {current_step}"

        # Render system prompt
        system_prompt = self.pm.render(
            "engineer", current_step=current_step, error="", plan=state.plan
        )

        # Initialize conversation with system prompt
        messages = [SystemMessage(content=system_prompt)]
        # Add existing messages from state if needed, but for CoderNode we usually want persistent context?
        # The state.messages might grow indefinitely if we just append.
        # For now, we start fresh for each task to keep context clean, but maybe we should pass state.messages?
        # Given the "task" based nature, starting fresh with relevant context (plan, todo) in system prompt is safer.
        # But if we want conversation history, we should use state.messages.
        # The previous implementation was one-shot. ReAct allows multi-turn.
        # Let's stick to fresh start per task to avoid context pollution, as the system prompt contains all necessary info.

        while retry_count < max_retries:
            try:
                # Invoke the agent
                result = await self.agent.ainvoke({"messages": messages})

                # Update messages with the conversation trace
                messages = result["messages"]

                # Validation Gate
                from worker.utils.file_validation import validate_node_output

                # Read current files to validate
                all_files = {}
                for f in [
                    "plan.md",
                    "todo.md",
                    "objectives.yaml",
                    "assembly_definition.yaml",
                ]:
                    with suppress(Exception):
                        all_files[f] = await self.fs.read_file(f)

                is_valid, validation_errors = validate_node_output("coder", all_files)

                if not is_valid:
                    error_msg = "Coder produced invalid output:\n" + "\n".join(
                        [f"- {e}" for e in validation_errors]
                    )
                    journal_entry += f"\nValidation failed (Attempt {retry_count + 1}): {validation_errors}"

                    # Append error to messages for the next retry
                    messages.append(HumanMessage(content=error_msg))
                    retry_count += 1
                    continue

                # Success
                journal_entry += f"\nSuccessfully executed step: {current_step}"

                # Mark TODO as done
                new_todo = todo.replace(
                    f"- [ ] {current_step}", f"- [x] {current_step}"
                )

                # Update state
                # We update 'messages' in state to persist the trace?
                # Or just keep it ephemeral? The state definition has 'messages'.
                # Let's persist it to allow debugging/history.

                return state.model_copy(
                    update={
                        "todo": new_todo,
                        "journal": state.journal + journal_entry,
                        "current_step": current_step,
                        "messages": messages,  # Persist the conversation
                        "turn_count": state.turn_count + 1,
                    }
                )

            except Exception as e:
                last_error = str(e)
                journal_entry += f"\nSystem error during execution: {last_error}"
                messages.append(HumanMessage(content=f"System error: {last_error}"))
                retry_count += 1

        journal_entry += f"\nFailed to complete step after {max_retries} retries."
        return state.model_copy(
            update={
                "journal": state.journal + journal_entry,
                "iteration": state.iteration + 1,
                "turn_count": state.turn_count + 1,
                "messages": messages,
            }
        )

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list."""
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                return line.strip().replace("- [ ]", "").strip()
        return None


# Factory function for LangGraph
@type_check
async def coder_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    node = CoderNode(worker_url=settings.spec_001_api_url, session_id=session_id)
    return await node(state)
