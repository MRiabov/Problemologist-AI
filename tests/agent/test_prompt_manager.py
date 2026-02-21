import pytest

from controller.agent.prompt_manager import PromptManager


def test_prompt_manager_render():
    pm = PromptManager()

    # Test planner template
    rendered_planner = pm.render("planner", task="build a car")
    assert "Mechanical Engineer" in rendered_planner
    assert rendered_planner.strip()

    # Test coder template
    rendered_coder = pm.render("coder", current_step="designing wheels")
    assert "Engineer" in rendered_coder
    assert rendered_coder.strip()


def test_prompt_manager_invalid_template():
    pm = PromptManager()
    with pytest.raises(ValueError, match="Template 'invalid' not found"):
        pm.render("invalid")
