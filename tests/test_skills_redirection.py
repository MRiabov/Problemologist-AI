import pytest
from pathlib import Path
from unittest.mock import MagicMock, patch
from src.environment.runtime import ToolRuntime

@pytest.fixture
def mock_runtime(tmp_path):
    return ToolRuntime(workspace_dir=str(tmp_path))

def test_view_file_dispatch(mock_runtime):
    """Test that view_file is correctly dispatched via _run_tool."""
    test_file = Path(mock_runtime.workspace_dir) / "test.txt"
    test_file.write_text("content", encoding="utf-8")

    # Should not raise ValueError
    result = mock_runtime._run_tool("view_file", {"path": "test.txt"})
    assert result == "content"

@patch("src.environment.runtime.Config")
def test_write_file_skills_redirection(mock_config, mock_runtime):
    """Test that write_file redirects docs/skills/ to Config.SKILLS_DIR."""
    # Setup mock skills dir
    mock_skills_dir = Path(mock_runtime.workspace_dir) / "mock_skills"
    mock_skills_dir.mkdir()
    mock_config.SKILLS_DIR = mock_skills_dir

    # Write to a skill
    skill_content = "New Skill Content"
    skill_rel_path = "my_skill.md"

    result = mock_runtime.write_file(skill_content, f"docs/skills/{skill_rel_path}")

    assert "Successfully wrote to skill" in result
    assert (mock_skills_dir / skill_rel_path).exists()
    assert (mock_skills_dir / skill_rel_path).read_text(encoding="utf-8") == skill_content

@patch("src.environment.runtime.Config")
def test_write_file_skills_security(mock_config, mock_runtime):
    """Test security check for directory traversal in skills."""
    mock_skills_dir = Path(mock_runtime.workspace_dir) / "mock_skills"
    mock_skills_dir.mkdir()
    mock_config.SKILLS_DIR = mock_skills_dir

    # Try to write outside skills dir
    result = mock_runtime.write_file("hacked", "docs/skills/../outside.txt")

    assert "attempts to traverse outside skills directory" in result
    assert not (mock_skills_dir.parent / "outside.txt").exists()

@patch("src.environment.runtime.Config")
def test_edit_file_skills_redirection(mock_config, mock_runtime):
    """Test that edit_file redirects docs/skills/ to Config.SKILLS_DIR."""
    mock_skills_dir = Path(mock_runtime.workspace_dir) / "mock_skills"
    mock_skills_dir.mkdir()
    mock_config.SKILLS_DIR = mock_skills_dir

    # Create existing skill
    skill_rel_path = "existing_skill.md"
    (mock_skills_dir / skill_rel_path).write_text("Old Content", encoding="utf-8")

    # Edit existing skill
    result = mock_runtime.edit_file(f"docs/skills/{skill_rel_path}", "Old", "New")

    assert "Successfully edited skill" in result
    assert (mock_skills_dir / skill_rel_path).read_text(encoding="utf-8") == "New Content"

@patch("src.environment.runtime.Config")
def test_edit_file_skills_missing(mock_config, mock_runtime):
    """Test editing a non-existent skill."""
    mock_skills_dir = Path(mock_runtime.workspace_dir) / "mock_skills"
    mock_skills_dir.mkdir()
    mock_config.SKILLS_DIR = mock_skills_dir

    result = mock_runtime.edit_file("docs/skills/missing.md", "Old", "New")

    assert "Error: Skill file docs/skills/missing.md does not exist" in result
