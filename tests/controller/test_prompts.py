import pytest
from controller.prompts import get_prompt


def test_get_planner_prompt():
    """
    Verifies that the planner prompt can be retrieved from prompts.yaml.
    """
    prompt = get_prompt("engineer.planner.system")
    assert "You are the Lead Mechanical Engineer (Planner)" in prompt
    assert "build123d_cad_drafting_skill" in prompt
    # Check for the specific architecture mention
    # assert "Claude Code" in prompt


def test_get_prompt_missing_key():
    """
    Verifies that a KeyError is raised for non-existent keys.
    """
    with pytest.raises(KeyError):
        get_prompt("non.existent.key")


def test_get_prompt_invalid_type():
    """
    Verifies that a ValueError is raised if the key points to a dict instead of a string.
    """
    with pytest.raises(ValueError):
        get_prompt("engineer.planner")
