from typing import Any

import dspy
import jinja2
import structlog

from shared.enums import AgentName
from shared.skills import build_skill_catalog_lines

from ..prompts import load_prompts

logger = structlog.get_logger(__name__)


class PromptManager:
    """Manager for Jinja2 templates for the agent."""

    def __init__(self):
        # Load templates from config/prompts.yaml
        data = load_prompts()

        # Mapping from our template names to prompts.yaml paths
        templates = {
            # Engineer Agent
            AgentName.ENGINEER_PLANNER.value: data["engineer"]["planner"]["system"],
            AgentName.ELECTRONICS_PLANNER.value: data["engineer"][
                "electronics_planner"
            ]["system"],
            AgentName.ENGINEER_CODER.value: data["engineer"]["engineer"]["system"],
            AgentName.ENGINEER_PLAN_REVIEWER.value: data["engineer"][
                "engineer_plan_reviewer"
            ]["system"],
            AgentName.ELECTRONICS_REVIEWER.value: data["engineer"][
                "electronics_reviewer"
            ]["system"],
            AgentName.ENGINEER_EXECUTION_REVIEWER.value: data["engineer"][
                "engineer_execution_reviewer"
            ]["system"],
            # Benchmark Generator
            AgentName.BENCHMARK_PLANNER.value: data["benchmark_generator"]["planner"][
                "system"
            ],
            AgentName.BENCHMARK_PLAN_REVIEWER.value: data["benchmark_generator"][
                "plan_reviewer"
            ]["system"],
            AgentName.BENCHMARK_CODER.value: data["benchmark_generator"]["coder"][
                "system"
            ],
            AgentName.BENCHMARK_REVIEWER.value: data["benchmark_generator"]["reviewer"][
                "system"
            ],
            # Subagents
            AgentName.COTS_SEARCH.value: data["subagents"]["cots_search"]["system"],
            AgentName.SKILL_AGENT.value: data["subagents"]["skill_learner"]["system"],
            AgentName.JOURNALLING_AGENT.value: data["subagents"]["token_compressor"][
                "system"
            ],
        }

        self.env = jinja2.Environment(loader=jinja2.DictLoader(templates))

    def render(self, template_name: str | AgentName, **kwargs: Any) -> str:
        """Render a template with the given context."""
        try:
            key = (
                template_name.value
                if isinstance(template_name, AgentName)
                else template_name
            )
            template = self.env.get_template(key)
            rendered = template.render(**kwargs).rstrip()
            skill_catalog = "\n".join(build_skill_catalog_lines())
            if rendered:
                return f"{rendered}\n\n{skill_catalog}\n"
            return f"{skill_catalog}\n"
        except jinja2.TemplateNotFound:
            raise ValueError(f"Template '{template_name}' not found")

    def get_prompt_value(self, key: str) -> Any:
        """Return a raw value from config/prompts.yaml using dot-separated keys."""
        data = load_prompts()
        value: Any = data
        for segment in key.split("."):
            if isinstance(value, dict) and segment in value:
                value = value[segment]
            else:
                raise KeyError(f"Prompt key '{key}' not found in config.")
        return value

    def load_compiled_program(
        self, agent_name: AgentName, program: dspy.Module
    ) -> dspy.Module:
        """
        Loads a compiled DSPy program from config/compiled_prompts/ if it exists.
        """
        from pathlib import Path

        # Look for compiled prompts in a standard location
        prompt_path = Path("config/compiled_prompts") / f"{agent_name.value}.json"

        if prompt_path.exists():
            try:
                program.load(str(prompt_path))
                logger.info(
                    "compiled_prompt_loaded",
                    agent_name=agent_name,
                    path=str(prompt_path),
                )
            except Exception as e:
                logger.error(
                    "compiled_prompt_load_failed",
                    agent_name=agent_name,
                    error=str(e),
                    session_id="system",
                )
        else:
            logger.debug("no_compiled_prompt_found", agent_name=agent_name)

        return program
