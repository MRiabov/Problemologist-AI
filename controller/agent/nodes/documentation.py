import dspy
import structlog

from controller.agent.state import AgentState
from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)

class DocumentationSignature(dspy.Signature):
    """
    Documentation node: Searches for technical documentation and skills.
    You will receive a query and must return concise, actionable snippets.
    Use your tools to read SKILL.md files and reference documentation.
    """
    query = dspy.InputField()
    skills_context = dspy.InputField()
    documentation_snippets = dspy.OutputField(desc="Concise snippets from documentation")

class DocumentationNode(BaseNode):
    """
    Documentation subagent: Provides targeted info from skills and reference docs.
    """
    async def __call__(self, state: AgentState, query: str) -> str:
        skills_context = self._get_skills_context()

        inputs = {
            "query": query,
            "skills_context": skills_context,
        }

        # For the documentation subagent, we use a simpler toolset
        from controller.agent.tools import get_common_tools

        prediction, _, _ = await self._run_program(
            program_cls=dspy.Predict, # Simple prediction for documentation lookup
            signature_cls=DocumentationSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_common_tools,
            validate_files=[],
            node_type="documentation_subagent",
        )

        if not prediction:
            return "Error: Documentation search failed."

        return getattr(prediction, "documentation_snippets", "No documentation found.")
