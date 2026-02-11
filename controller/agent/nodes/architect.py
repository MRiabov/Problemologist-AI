from pathlib import Path

from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

from ..config import settings
from ..prompt_manager import PromptManager
from ..state import AgentState


@type_check
async def architect_node(state: AgentState) -> AgentState:
    """
    Architect node: Analyzes the task and creates plan.md and todo.md.
    """
    pm = PromptManager()

    # T006: Read skills (local controller FS is fine for skills)
    skills_dir = Path(".agent/skills")
    skills = []
    if skills_dir.exists():
        skills = [d.name for d in skills_dir.iterdir() if d.is_dir()]

    skills_context = "\n".join([f"- {s}" for s in skills])

    # T005: Invoke LLM
    prompt_text = pm.render("architect", task=state.task, skills=skills_context)

    # Using a standard LLM configuration for the agent
    llm = ChatOpenAI(model="gpt-4o", temperature=0)

    # We pass the prompt as a human message for simplicity in this skeleton
    response = await llm.ainvoke([HumanMessage(content=prompt_text)])
    content = str(response.content)

    # T007 & T008: Parse output and create artifacts
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

    # Write files to the worker session workspace
    session_id = state.session_id or settings.default_session_id
    worker_client = WorkerClient(
        base_url=settings.spec_001_api_url, session_id=session_id
    )
    fs = RemoteFilesystemMiddleware(worker_client)

    await fs.write_file("plan.md", plan_content)
    await fs.write_file("todo.md", todo_content)

    # Emit SubmissionValidationEvent
    artifacts_present = []
    errors = []
    if plan_content.strip():
        artifacts_present.append("plan.md")
    else:
        errors.append("plan.md is empty")

    if todo_content.strip():
        artifacts_present.append("todo.md")
    else:
        errors.append("todo.md is empty")

    await record_worker_events(
        episode_id=session_id,
        events=[
            SubmissionValidationEvent(
                artifacts_present=artifacts_present,
                verification_passed=len(errors) == 0,
                reasoning_trace_quality=1.0,  # Placeholder
                errors=errors,
            )
        ],
    )

    return state.model_copy(
        update={
            "plan": plan_content,
            "todo": todo_content,
        }
    )
