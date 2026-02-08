from typing import Any

import jinja2
from controller.prompts import get_prompt


class PromptManager:
    """Manager for Jinja2 templates for the agent, loading from prompts.yaml."""

    def __init__(self):
        # We'll load the templates on demand or during initialization
        self._templates = self._load_all_templates()
        self.env = jinja2.Environment(loader=jinja2.DictLoader(self._templates))

    def _load_all_templates(self) -> dict[str, str]:
        """Loads all templates from the configuration."""
        templates = {}

        # Mapping of internal template names to YAML keys
        mapping = {
            "architect": "cad_agent.planner.system",
            "engineer": "cad_agent.actor.system",
            "critic": "cad_agent.critic.system",
            "sidecar": "cad_agent.sidecar.system",
        }

        for name, key in mapping.items():
            try:
                templates[name] = get_prompt(key)
            except (KeyError, ValueError) as e:
                # Fallback to hardcoded defaults if not found in YAML
                # This ensures the agent can still function if the YAML is missing some keys
                templates[name] = self._get_default_template(name)

        return templates

    def _get_default_template(self, name: str) -> str:
        """Fallback hardcoded templates."""
        defaults = {
            "architect": """You are the Architect.
Your goal is to plan the following task: {{ task }}

Available skills:
{{ skills }}

Instructions:
1. Analyze the task and available skills.
2. Create a high-level execution plan.
3. Create a detailed TODO list of specific implementation steps.

Output your response in two sections:
# PLAN
<your plan here>

# TODO
- [ ] <step 1>
- [ ] <step 2>
""",
            "engineer": """You are the Engineer.
Implement the following step: {{ current_step }}

Execution Plan context:
{{ plan }}

{% if error %}
Your previous attempt failed with this error:
{{ error }}
Please fix the code and try again.
{% endif %}

Instructions:
1. Write a standalone Python script to accomplish the step.
2. Use only available tools and libraries.
3. Output ONLY the Python code inside a markdown code block.
""",
            "critic": """You are the Critic.
Evaluate the implementation of the task: {{ task }}

Execution Journal:
{{ journal }}

Simulation Report:
{{ sim_report }}

Manufacturability Report:
{{ mfg_report }}

Instructions:
1. Check if the simulation passed.
2. Check if the part is manufacturable and within budget.
3. Decide whether to APPROVE, REJECT_PLAN (if the whole strategy is wrong), or REJECT_CODE (if it's just a small bug).

You MUST output your review as a valid Markdown file with YAML frontmatter.

Your output format MUST be exactly:
---
decision: "approve" | "reject_code" | "reject_plan"
required_fixes:
  - "Short bullet point 1"
  - "Short bullet point 2"
---

# Review Analysis
[Your detailed reasoning and feedback here...]
""",
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
        }
        return defaults.get(name, "")

    def render(self, template_name: str, **kwargs: Any) -> str:
        """Render a template with the given context."""
        try:
            template = self.env.get_template(template_name)
            return template.render(**kwargs)
        except jinja2.TemplateNotFound:
            raise ValueError(f"Template '{template_name}' not found")
