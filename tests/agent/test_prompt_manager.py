import pytest
from src.agent.prompt_manager import PromptManager

def test_prompt_manager_render():
    pm = PromptManager()
    
    # Test architect template
    rendered_architect = pm.render("architect", task="build a car")
    assert "Architect" in rendered_architect
    assert "build a car" in rendered_architect
    
    # Test engineer template
    rendered_engineer = pm.render("engineer", current_step="designing wheels")
    assert "Engineer" in rendered_engineer
    assert "designing wheels" in rendered_engineer

def test_prompt_manager_invalid_template():
    pm = PromptManager()
    with pytest.raises(ValueError, match="Template 'invalid' not found"):
        pm.render("invalid")
