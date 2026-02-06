import jinja2
from typing import Any

class PromptManager:
    """Manager for Jinja2 templates for the agent."""
    
    def __init__(self):
        self.env = jinja2.Environment(
            loader=jinja2.DictLoader({
                "architect": "You are the Architect. Plan the task: {{ task }}",
                "engineer": "You are the Engineer. Implement the current step: {{ current_step }}",
            })
        )

    def render(self, template_name: str, **kwargs: Any) -> str:
        """Render a template with the given context."""
        try:
            template = self.env.get_template(template_name)
            return template.render(**kwargs)
        except jinja2.TemplateNotFound:
            raise ValueError(f"Template '{template_name}' not found")
