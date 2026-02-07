import os
from pathlib import Path
from typing import Any, Dict

from langchain_openai import ChatOpenAI
from langchain_core.messages import HumanMessage

from ..prompt_manager import PromptManager
from ..state import AgentState
from src.shared.type_checking import type_check


@type_check
def architect_node(state: AgentState) -> AgentState:
    """
    Architect node: Analyzes the task and creates plan.md and todo.md.
    """
    pm = PromptManager()

    # T006: Read skills
    skills_dir = Path(".agent/skills")
    skills = []
    if skills_dir.exists():
        skills = [
            d.name
            for d in skills_dir.iterdir()
            if d.is_dir()
        ]

    skills_context = "\n".join([f"- {s}" for s in skills])

    # T005: Invoke LLM
    prompt_text = pm.render("architect", task=state.task, skills=skills_context)

    # Using a standard LLM configuration for the agent
    llm = ChatOpenAI(model="gpt-4o", temperature=0)

    # We pass the prompt as a human message for simplicity in this skeleton
    response = llm.invoke([HumanMessage(content=prompt_text)])
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

    # Write files to the workspace root
    with open("plan.md", "w") as f:
        f.write(plan_content)

    with open("todo.md", "w") as f:
        f.write(todo_content)

    return state.model_copy(
        update={
            "plan": plan_content,
            "todo": todo_content,
        }
    )
