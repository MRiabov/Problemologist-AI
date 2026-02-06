import json
from typing import Any, Dict, Optional

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from ..prompt_manager import PromptManager
from ..state import AgentState
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
from src.controller.clients.worker import WorkerClient


class CriticNode:
    """
    Critic node: Evaluates the Engineer's output based on simulation and workbench reports.
    """

    def __init__(
        self,
        worker_url: str = "http://worker:8001",
        session_id: str = "default-session",
    ):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        self.fs = RemoteFilesystemMiddleware(self.worker_client)

    async def __call__(self, state: AgentState) -> Dict[str, Any]:
        # T016 & T017: Read artifacts
        sim_report = await self._read_json_artifact("simulation_report.json")
        mfg_report = await self._read_artifact("workbench_report.md")

        # T015: Use LLM to evaluate success
        prompt = self.pm.render(
            "critic",
            task=state.task,
            journal=state.journal,
            sim_report=json.dumps(sim_report, indent=2)
            if sim_report
            else "No simulation report found.",
            mfg_report=mfg_report or "No manufacturability report found.",
        )

        response = await self.llm.ainvoke([HumanMessage(content=prompt)])
        content = str(response.content)

        # T018: Decision logic
        # Expecting format:
        # DECISION: APPROVE | REJECT_PLAN | REJECT_CODE
        # FEEDBACK: <feedback>

        decision = "REJECT_CODE"
        feedback = "Failed to parse critic decision."

        if "DECISION: APPROVE" in content:
            decision = "APPROVE"
        elif "DECISION: REJECT_PLAN" in content:
            decision = "REJECT_PLAN"
        elif "DECISION: REJECT_CODE" in content:
            decision = "REJECT_CODE"
        elif "DECISION: REJECT" in content:
            decision = "REJECT_CODE"  # Default reject

        if "FEEDBACK:" in content:
            feedback = content.split("FEEDBACK:")[1].strip()
        else:
            feedback = content.strip()

        journal_entry = f"\nCritic Decision: {decision}\nFeedback: {feedback}"

        status_map = {
            "APPROVE": "approved",
            "REJECT_PLAN": "plan_rejected",
            "REJECT_CODE": "code_rejected",
        }

        return {
            "status": status_map.get(decision, "code_rejected"),
            "feedback": feedback,
            "journal": state.journal + journal_entry,
        }

    async def _read_artifact(self, path: str) -> Optional[str]:
        try:
            return await self.fs.read_file(path)
        except Exception:
            return None

    async def _read_json_artifact(self, path: str) -> Optional[Dict[str, Any]]:
        content = await self._read_artifact(path)
        if content:
            try:
                return json.loads(content)
            except json.JSONDecodeError:
                return None
        return None


from ..config import settings


# Factory function for LangGraph
async def critic_node(state: AgentState) -> Dict[str, Any]:
    node = CriticNode(
        worker_url=settings.spec_001_api_url, session_id=settings.default_session_id
    )
    return await node(state)
