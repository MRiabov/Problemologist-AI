import logging
from typing import Any, Dict, List, Optional

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from ..prompt_manager import PromptManager
from ..state import AgentState
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.clients.worker import WorkerClient

from shared.type_checking import type_check

logger = logging.getLogger(__name__)


@type_check
class EngineerNode:
    """
    Engineer node: Picks a task from TODO, writes code, executes it, and fixes errors.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        # T011 & T012: Initialize middleware/client
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.fs = RemoteFilesystemMiddleware(self.worker_client)

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the engineer node logic."""
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
                execution_result = await self.fs.run_command(code)
                stdout = execution_result.get("stdout", "")
                stderr = execution_result.get("stderr", "")
                exit_code = execution_result.get("exit_code", 0)

                if exit_code == 0:
                    journal_entry += f"\nSuccessfully executed step: {current_step}"
                    # Mark TODO as done (simple string replacement for prototype)
                    new_todo = todo.replace(
                        f"- [ ] {current_step}", f"- [x] {current_step}"
                    )
                    return state.model_copy(
                        update={
                            "todo": new_todo,
                            "journal": state.journal + journal_entry,
                            "current_step": current_step,
                        }
                    )
                else:
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


from ..config import settings


# Factory function for LangGraph
@type_check
async def engineer_node(state: AgentState) -> AgentState:
    node = EngineerNode(
        worker_url=settings.spec_001_api_url, session_id=settings.default_session_id
    )
    return await node(state)
