from typing import Any
import dspy
import jinja2
import structlog

from ..prompts import load_prompts

logger = structlog.get_logger(__name__)


class PromptManager:
    """Manager for Jinja2 templates for the agent."""

    def __init__(self):
        # Load templates from config/prompts.yaml
        data = load_prompts()

        # Mapping from our template names to prompts.yaml paths
        templates = {
            "architect": data["engineer"]["planner"]["system"],
            "engineer": data["engineer"]["engineer"]["system"],
            "electronics_engineer": data["engineer"]["electronics_engineer"]["system"],
            "critic": data["engineer"]["critic"]["system"],
            "benchmark_planner": data["benchmark_generator"]["planner"]["system"],
            "benchmark_coder": data["benchmark_generator"]["coder"]["system"],
            "benchmark_reviewer": data["benchmark_generator"]["reviewer"]["system"],
            "cots_search": data["subagents"]["cots_search"]["system"],
        }

        # Add fallbacks for things not in prompts.yaml yet
        templates.update(
            {
                "sidecar": """You are the Sidecar Learner. 
Analyze the execution journal for the task: {{ task }}

Journal:
{{ journal }}

Instructions:
1. Identify any repeated struggles or successful breakthroughs.
2. If a reusable pattern or "trick" is found, draft a new skill.
3. If no significant new knowledge is found, say "No new skills identified."

If suggesting a skill, output in this format:
SKILL: <Short Title>
CONTENT:
# <Title>
## Problem
<Describe the problem encountered>
## Solution
<Describe the solution or best practice>
## Example
```python
<Example code>
```
""",
                "git_resolver": """You are a Git Merge Conflict Resolver.
You have been given a file with git conflict markers (<<<<<<<, =======, >>>>>>>).

Your task is to resolve the conflict intelligently.
- The 'HEAD' (or 'ours') version is usually the Agent's new code or skill.
- The 'remote' (or 'theirs') version is the existing code in the repository.

Guidelines:
1. Prefer the Agent's new content if it adds new value or fixes a problem.
2. If the remote version has critical updates, try to merge them.
3. Ensure the final code is syntactically correct and logical.
4. Remove all conflict markers.

Conflicting File Content:
{{ content }}

Output ONLY the resolved content. Do not include markdown code blocks (```) unless they are part of the file content itself.
""",
            }
        )

        self.env = jinja2.Environment(loader=jinja2.DictLoader(templates))

    def render(self, template_name: str, **kwargs: Any) -> str:
        """Render a template with the given context."""
        try:
            template = self.env.get_template(template_name)
            return template.render(**kwargs)
        except jinja2.TemplateNotFound:
            raise ValueError(f"Template '{template_name}' not found")

    def load_compiled_program(
        self, agent_name: str, program: dspy.Module
    ) -> dspy.Module:
        """
        Loads a compiled DSPy program from config/compiled_prompts/ if it exists.
        """
        from pathlib import Path

        # Look for compiled prompts in a standard location
        prompt_path = Path("config/compiled_prompts") / f"{agent_name}.json"

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
                    "compiled_prompt_load_failed", agent_name=agent_name, error=str(e)
                )
        else:
            logger.debug("no_compiled_prompt_found", agent_name=agent_name)

        return program
