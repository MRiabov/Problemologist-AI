import pytest

from controller.agent.prompt_manager import PromptManager


def test_prompt_manager_render():
    pm = PromptManager()

    # Test architect template
    rendered_architect = pm.render("architect", task="build a car")
    assert "Planner" in rendered_architect

    # Test engineer template
    rendered_engineer = pm.render("engineer", current_step="designing wheels")
    assert "CAD Engineer" in rendered_engineer


def test_prompt_manager_invalid_template():
    pm = PromptManager()
    with pytest.raises(ValueError, match="Template 'invalid' not found"):
        pm.render("invalid")
