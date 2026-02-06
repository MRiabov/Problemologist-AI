import json
import logging
from typing import Any

from langchain_core.messages import BaseMessage, HumanMessage, ToolMessage
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from temporalio.client import Client

from src.controller.clients.worker import WorkerClient
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)


class EngineerNode:
    """
    Engineer node: Picks a task from TODO, writes code, executes it, and fixes errors.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
        temporal_client: Client | None = None,
    ):
        self.pm = PromptManager()
        # Bind tools to the LLM
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.fs = RemoteFilesystemMiddleware(
            self.worker_client, temporal_client=temporal_client
        )

        # Define tools using the fs middleware
        @tool
        async def list_files(path: str = "/") -> str:
            """List files in the remote environment."""
            try:
                files = await self.fs.list_files(path)
                return json.dumps(files, indent=2)
            except Exception as e:
                return f"Error listing files: {str(e)}"

        @tool
        async def read_file(path: str) -> str:
            """Read a file from the remote environment."""
            try:
                return await self.fs.read_file(path)
            except Exception as e:
                return f"Error reading file: {str(e)}"

        self.tools = [list_files, read_file]
        self.llm_with_tools = self.llm.bind_tools(self.tools)

    async def __call__(self, state: AgentState) -> dict[str, Any]:
        """Execute the engineer node logic."""
        todo = state.todo
        current_step = self._get_next_step(todo)

        if not current_step:
            return {
                "journal": state.journal + "\nAgend Engineer: No more steps in TODO."
            }

        # Render initial prompt
        prompt = self.pm.render(
            "engineer", current_step=current_step, error="", plan=state.plan
        )

        # Conversation history for this step
        messages: list[BaseMessage] = [HumanMessage(content=prompt)]

        max_retries = 3
        retry_count = 0
        journal_entry = f"\nStarting step: {current_step}"

        while retry_count < max_retries:
            # Invoke LLM
            response = await self.llm_with_tools.ainvoke(messages)
            messages.append(response)

            # Handle Tool Calls
            if response.tool_calls:
                for tool_call in response.tool_calls:
                    # Execute tool
                    tool_name = tool_call["name"]
                    tool_args = tool_call["args"]
                    tool_result = "Unknown tool"

                    if tool_name == "list_files":
                        try:
                            res = await self.fs.list_files(tool_args.get("path", "/"))
                            tool_result = json.dumps(res)
                        except Exception as e:
                            tool_result = str(e)
                    elif tool_name == "read_file":
                        try:
                            tool_result = await self.fs.read_file(tool_args.get("path"))
                        except Exception as e:
                            tool_result = str(e)

                    messages.append(
                        ToolMessage(tool_call_id=tool_call["id"], content=tool_result)
                    )
                # Loop back to let LLM process tool output
                continue

            # Handle Code Generation
            code = self._extract_code(str(response.content))
            if code:
                # Execute Code
                try:
                    # Using worker_client.execute_python as per plan which is
                    # wrapped by fs.run_command alias
                    execution_result = await self.fs.run_command(code)
                    stdout = execution_result.get("stdout", "")
                    stderr = execution_result.get("stderr", "")
                    exit_code = execution_result.get("exit_code", 0)

                    if exit_code == 0:
                        journal_entry += f"\nSuccessfully executed step: {current_step}"
                        new_todo = todo.replace(
                            f"- [ ] {current_step}", f"- [x] {current_step}"
                        )
                        return {
                            "todo": new_todo,
                            "journal": state.journal + journal_entry,
                            "current_step": current_step,
                        }

                    error_msg = stderr or stdout or "Unknown execution error"
                    journal_entry += (
                        f"\nExecution failed (Attempt {retry_count + 1}): {error_msg}"
                    )

                    # Feed back error to LLM
                    feedback_prompt = self.pm.render(
                        "engineer",
                        current_step=current_step,
                        error=error_msg,
                        plan=state.plan,
                    )
                    messages.append(HumanMessage(content=feedback_prompt))
                    retry_count += 1
                except Exception as e:
                    error_msg = f"System error: {str(e)}"
                    journal_entry += f"\n{error_msg}"
                    messages.append(
                        HumanMessage(
                            content=f"System error executing code: {error_msg}"
                        )
                    )
                    retry_count += 1
            else:
                # No code and no tools? Ask again.
                messages.append(
                    HumanMessage(
                        content="Please output Python code to execute the step."
                    )
                )
                retry_count += 1

        journal_entry += f"\nFailed to complete step after {max_retries} attempts."
        return {
            "journal": state.journal + journal_entry,
            "iteration": state.iteration + 1,
        }

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list."""
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                return line.strip().replace("- [ ]", "").strip()
        return None

    def _extract_code(self, content: str) -> str | None:
        """Extract Python code from markdown block if present."""
        if "```python" in content:
            return content.split("```python")[1].split("```")[0].strip()
        if "```" in content:
            # Fallback for generic blocks
            return content.split("```")[1].split("```")[0].strip()
        return None


# Factory function for LangGraph
async def engineer_node(state: AgentState) -> dict[str, Any]:
    # In a real app, these would come from config/env
    temporal_client = None
    try:
        temporal_client = await Client.connect("localhost:7233")
    except Exception as e:
        logger.warning(
            f"Could not connect to Temporal: {e}. Falling back to sync execution."
        )

    node = EngineerNode(temporal_client=temporal_client)
    return await node(state)
