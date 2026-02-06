import logging
import asyncio
from pathlib import Path
from typing import Any, Set

from langchain_core.messages import HumanMessage
# Lazy import to avoid side effects
# from langchain_openai import ChatOpenAI

from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)

# Keep strong references to background tasks to prevent GC
# See: https://docs.python.org/3/library/asyncio-task.html#asyncio.create_task
_background_tasks: Set[asyncio.Task] = set()


class SidecarNode:
    """
    Sidecar Learner node: Analyzes the journal to suggest new skills.
    """

    def __init__(self, suggested_skills_dir: str = "suggested_skills"):
        from langchain_openai import ChatOpenAI

        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)

    async def __call__(self, state: AgentState) -> None:
        """Execute the sidecar node logic in background."""
        try:
            # T021: Parse Journal for patterns
            prompt = self.pm.render("sidecar", journal=state.journal, task=state.task)

            response = await self.llm.ainvoke([HumanMessage(content=prompt)])
            content = str(response.content)

            # T022: Skill extraction logic
            # Expecting format:
            # SKILL: <Title>
            # CONTENT: <Markdown content>

            if "SKILL:" in content and "CONTENT:" in content:
                parts = content.split("CONTENT:")
                title = parts[0].replace("SKILL:", "").strip().lower().replace(" ", "_")
                skill_content = parts[1].strip()

                file_path = self.suggested_skills_dir / f"{title}.md"
                # Use Path.open (aliased via simple open on Path object or write_text)
                file_path.write_text(skill_content)
                logger.info(f"Suggested new skill: {title}")
            else:
                logger.info("No new skills identified by Sidecar.")

        except Exception as e:
            logger.error(f"Error in Sidecar background task: {e}", exc_info=True)


# Factory function for LangGraph
async def sidecar_node(state: AgentState) -> dict[str, Any]:
    node = SidecarNode()
    # Launch as background task
    task = asyncio.create_task(node(state))

    # Add to set to prevent GC
    _background_tasks.add(task)
    task.add_done_callback(_background_tasks.discard)

    # Return immediately without modifying state
    return {}
