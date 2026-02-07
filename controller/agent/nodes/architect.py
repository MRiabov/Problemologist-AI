import logging

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
        """Execute the architect node logic."""
        # T006: Read skills from worker
        skills = []
        try:
            # Check for base skills in /skills
            base_skills = await self.worker_client.list_files("/skills")
            skills.extend([s.name for s in base_skills if s.is_dir])

            # Check for learned skills in suggested_skills
            learned_skills = await self.worker_client.list_files("suggested_skills")
            skills.extend([s.name for s in learned_skills if s.name.endswith(".md")])
        except Exception as e:
            logger.warning(f"Could not fetch skills from worker: {e}")

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

        # Write files to the worker workspace root
        try:
            await self.worker_client.write_file("plan.md", plan_content)
            await self.worker_client.write_file("todo.md", todo_content)
        except Exception as e:
            logger.error(f"Failed to write plan/todo to worker: {e}")

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
