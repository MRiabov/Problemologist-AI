from enum import Enum, StrEnum
import json
import re
import yaml
from typing import Any, Dict, Optional

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from ..prompt_manager import PromptManager
from ..state import AgentState, AgentStatus
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
from src.controller.clients.worker import WorkerClient
from src.shared.type_checking import type_check


class CriticDecision(StrEnum):
    APPROVE = "APPROVE"
    REJECT_PLAN = "REJECT_PLAN"
    REJECT_CODE = "REJECT_CODE"


@type_check
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

    async def __call__(self, state: AgentState) -> AgentState:
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
        # ---
        # decision: "approve" | "reject_plan" | "reject_code"
        # required_fixes: ...
        # ---
        # ...

        decision = CriticDecision.REJECT_CODE
        feedback = "Failed to parse critic decision."

        try:
            # Flexible match for frontmatter (start of string or after whitespace)
            match = re.search(r"^---\n(.*?)\n---\n(.*)", content, re.DOTALL | re.MULTILINE)
            if match:
                yaml_block = match.group(1)
                body = match.group(2)
                data = yaml.safe_load(yaml_block)
                
                decision_str = data.get("decision", "").upper()
                if decision_str == "APPROVE":
                    decision = CriticDecision.APPROVE
                elif decision_str == "REJECT_PLAN":
                    decision = CriticDecision.REJECT_PLAN
                elif decision_str == "REJECT_CODE":
                    decision = CriticDecision.REJECT_CODE
                
                required_fixes = data.get("required_fixes", [])
                if required_fixes:
                    fixes_text = "\n".join([f"- {fix}" for fix in required_fixes])
                    feedback = f"{body.strip()}\n\nRequired Fixes:\n{fixes_text}"
                else:
                    feedback = body.strip()
            else:
                 feedback = f"Critic failed to produce valid frontmatter. Raw output:\n{content}"
        except Exception as e:
            feedback = f"Error parsing critic output: {e}\nRaw output:\n{content}"

        journal_entry = f"\nCritic Decision: {decision.value}\nFeedback: {feedback}"

        status_map = {
            CriticDecision.APPROVE: AgentStatus.APPROVED,
            CriticDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            CriticDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
        }

        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.CODE_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
            }
        )

    async def _read_artifact(self, path: str) -> str | None:
        try:
            return await self.fs.read_file(path)
        except Exception:
            return None

    async def _read_json_artifact(self, path: str) -> dict[str, Any] | None:
        content = await self._read_artifact(path)
        if content:
            try:
                return json.loads(content)
            except json.JSONDecodeError:
                return None
        return None


from ..config import settings


# Factory function for LangGraph
@type_check
async def critic_node(state: AgentState) -> AgentState:
    node = CriticNode(
        worker_url=settings.spec_001_api_url, session_id=settings.default_session_id
    )
    return await node(state)
