import logging
from pathlib import Path

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from controller.clients.worker import WorkerClient
from shared.type_checking import type_check

from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)


@type_check
class ArchitectNode:
    """
    Architect node: Analyzes the task and creates plan.md and todo.md.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)

    async def __call__(self, state: AgentState) -> AgentState:
        # T006: Read skills
        skills = []
        try:
            # Fetch skills from mounted /skills (read-only system skills)
            # Note: worker mounts skills at /skills
            mounted_skills = await self.worker_client.list_files("/skills")
            skills.extend([f.name for f in mounted_skills if f.is_dir])
        except Exception as e:
            # Fallback or log warning if worker is unreachable or /skills empty
            logger.warning(f"Failed to fetch skills from /skills: {e}")

        try:
            # Fetch suggested skills from workspace (writable learned skills)
            suggested_skills = await self.worker_client.list_files("suggested_skills")
            # Suggested skills might be files (.md) or directories
            skills.extend(
                [
                    f.name.replace(".md", "")
                    for f in suggested_skills
                    if f.name.endswith(".md")
                ]
            )
        except Exception as e:
            # suggested_skills might not exist yet
            logger.debug(f"Failed to fetch suggested skills: {e}")

        skills_context = "\n".join([f"- {s}" for s in skills])

        # T005: Invoke LLM
        prompt_text = self.pm.render("architect", task=state.task, skills=skills_context)

        # We pass the prompt as a human message for simplicity in this skeleton
        response = await self.llm.ainvoke([HumanMessage(content=prompt_text)])
        content = str(response.content)

        # T007 & T008: Parse output and create artifacts
        # Expecting a format like:
        # # PLAN
        # ...
        # # TODO
        # ...

        plan_content = ""
        todo_content = ""

        if "# PLAN" in content and "# TODO" in content:
            parts = content.split("# TODO")
            plan_content = parts[0].replace("# PLAN", "").strip()
            todo_content = parts[1].strip()
        else:
            # Fallback if LLM doesn't follow instructions perfectly
            plan_content = content
            todo_content = "- [ ] Implement the plan"

        # Write files to the workspace (remote) via worker client
        # This ensures artifacts are visible in the shared workspace
        await self.worker_client.write_file("plan.md", plan_content)
        await self.worker_client.write_file("todo.md", todo_content)

        return state.model_copy(
            update={
                "plan": plan_content,
                "todo": todo_content,
            }
        )


from ..config import settings


# Factory function for LangGraph
@type_check
async def architect_node(state: AgentState) -> AgentState:
    node = ArchitectNode(
        worker_url=settings.spec_001_api_url, session_id=settings.default_session_id
    )
    return await node(state)
