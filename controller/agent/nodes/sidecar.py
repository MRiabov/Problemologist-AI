import logging

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from controller.clients.worker import WorkerClient
from shared.type_checking import type_check

from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)


@type_check
class SidecarNode:
    """
    Sidecar Learner node: Analyzes the journal to suggest new skills.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
        suggested_skills_dir: str = "suggested_skills",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.suggested_skills_dir = suggested_skills_dir

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the sidecar node logic."""
        # T021: Parse Journal for patterns
        prompt = self.pm.render("sidecar", journal=state.journal, task=state.task)

        response = await self.llm.ainvoke([HumanMessage(content=prompt)])
        content = str(response.content)

        # T022: Skill extraction logic
        # Expecting format:
        # SKILL: <Title>
        # CONTENT: <Markdown content>

        suggested_skill = None
        if "SKILL:" in content and "CONTENT:" in content:
            parts = content.split("CONTENT:")
            title = parts[0].replace("SKILL:", "").strip().lower().replace(" ", "_")
            skill_content = parts[1].strip()

            # Write to workspace via worker client (remote)
            file_path = f"{self.suggested_skills_dir}/{title}.md"
            try:
                success = await self.worker_client.write_file(file_path, skill_content)
                if success:
                    suggested_skill = title
                    logger.info(f"Suggested new skill: {title}")
                else:
                    logger.error(f"Failed to write suggested skill: {title}")
            except Exception as e:
                logger.error(f"Error writing suggested skill: {e}")

        journal_entry = f"\nSidecar Learner: {'Suggested skill ' + suggested_skill if suggested_skill else 'No new skills identified.'}"

        return state.model_copy(update={"journal": state.journal + journal_entry})


from ..config import settings


# Factory function for LangGraph
@type_check
async def sidecar_node(state: AgentState) -> AgentState:
    node = SidecarNode(
        worker_url=settings.spec_001_api_url, session_id=settings.default_session_id
    )
    return await node(state)
