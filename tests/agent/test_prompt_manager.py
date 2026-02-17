import pytest

from controller.agent.prompt_manager import PromptManager


def test_prompt_manager_render():
    pm = PromptManager()

    # Test architect template
    rendered_architect = pm.render("architect", task="build a car")
    assert "Mechanical Engineer" in rendered_architect
    # The current template doesn't include the task in the system prompt,
    # it's usually passed in a separate message.
    # So we just verify it renders successfully.
    assert rendered_architect.strip()

    # Test engineer template
    rendered_engineer = pm.render("engineer", current_step="designing wheels")
    assert "Engineer" in rendered_engineer
    # Verify it renders successfully
    assert rendered_engineer.strip()


def test_prompt_manager_invalid_template():
    pm = PromptManager()
    with pytest.raises(ValueError, match="Template 'invalid' not found"):
        pm.render("invalid")
