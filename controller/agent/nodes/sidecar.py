import logging
from pathlib import Path

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from shared.type_checking import type_check

from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)


@type_check
class SidecarNode:
    """
    Sidecar Learner node: Analyzes the journal to suggest new skills.
    """

    def __init__(self, suggested_skills_dir: str = "suggested_skills"):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)

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

            file_path = self.suggested_skills_dir / f"{title}.md"
            with open(file_path, "w") as f:
                f.write(skill_content)
            suggested_skill = title
            logger.info(f"Suggested new skill: {title}")

        journal_entry = f"\nSidecar Learner: {'Suggested skill ' + suggested_skill if suggested_skill else 'No new skills identified.'}"

        return state.model_copy(update={"journal": state.journal + journal_entry})


# Factory function for LangGraph
@type_check
async def sidecar_node(state: AgentState) -> AgentState:
    node = SidecarNode()
    return await node(state)
