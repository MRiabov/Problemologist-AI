import jinja2
from typing import Any


class PromptManager:
    """Manager for Jinja2 templates for the agent."""

    def __init__(self):
        self.env = jinja2.Environment(
            loader=jinja2.DictLoader(
                {
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
1. You may use available tools `list_files` and `read_file` to inspect the environment if needed.
2. Provide the final solution as a standalone Python script.
3. Output the Python code inside a markdown code block (```python ... ```).
""",
                }
            )
        )

    def render(self, template_name: str, **kwargs: Any) -> str:
        """Render a template with the given context."""
        try:
            template = self.env.get_template(template_name)
            return template.render(**kwargs)
        except jinja2.TemplateNotFound:
            raise ValueError(f"Template '{template_name}' not found")
