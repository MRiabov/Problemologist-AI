import os
import logging
from typing import Any, Dict, Optional

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from ..prompt_manager import PromptManager
from ..state import AgentState
from src.shared.type_checking import type_check

logger = logging.getLogger(__name__)


@type_check
class SidecarNode:
    """
    Sidecar Learner node: Analyzes the journal to suggest new skills.
    """

    def __init__(self, suggested_skills_dir: str = "suggested_skills"):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.suggested_skills_dir = suggested_skills_dir
        os.makedirs(self.suggested_skills_dir, exist_ok=True)

    async def __call__(self, state: AgentState) -> Dict[str, Any]:
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

            file_path = os.path.join(self.suggested_skills_dir, f"{title}.md")
            with open(file_path, "w") as f:
                f.write(skill_content)
            suggested_skill = title
            logger.info(f"Suggested new skill: {title}")

        journal_entry = f"\nSidecar Learner: {'Suggested skill ' + suggested_skill if suggested_skill else 'No new skills identified.'}"

        return {"journal": state.journal + journal_entry}


# Factory function for LangGraph
@type_check
async def sidecar_node(state: AgentState) -> Dict[str, Any]:
    node = SidecarNode()
    return await node(state)
