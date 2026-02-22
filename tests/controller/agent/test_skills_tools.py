import pytest
from pathlib import Path
from unittest.mock import MagicMock, patch
import os

from controller.agent.tools import get_common_tools

@pytest.mark.asyncio
async def test_skills_tools(tmp_path):
    # Setup temporary skills directory
    mock_skills_dir = tmp_path / "skills"
    mock_skills_dir.mkdir()

    # Create a dummy skill
    skill_name = "test_skill"
    skill_dir = mock_skills_dir / skill_name
    skill_dir.mkdir()

    # Create files in skill
    (skill_dir / "SKILL.md").write_text("Test content", encoding="utf-8")
    (skill_dir / "script.py").write_text("print('hello')", encoding="utf-8")
    (skill_dir / ".hidden").write_text("hidden", encoding="utf-8")

    # Create a file outside the skills directory to test traversal
    outside_file = tmp_path / "outside.txt"
    outside_file.write_text("secret", encoding="utf-8")

    # Mock RemoteFilesystemMiddleware (not used by skill tools but required by get_common_tools)
    mock_fs = MagicMock()

    # Patch SKILLS_DIR in the module where tools are defined
    # We patch it on the module object 'controller.agent.tools'
    with patch("controller.agent.tools.SKILLS_DIR", mock_skills_dir):
        tools = get_common_tools(mock_fs, "session-123")

        # Extract tools by name for easy access
        tool_map = {t.__name__: t for t in tools}
        list_skill_files = tool_map.get("list_skill_files")
        read_skill = tool_map.get("read_skill")

        assert list_skill_files is not None
        assert read_skill is not None

        # Test list_skill_files
        files = await list_skill_files(skill_name)
        assert "SKILL.md" in files
        assert "script.py" in files
        assert ".hidden" not in files

        # Test list_skill_files for non-existent skill
        error = await list_skill_files("non_existent")
        assert "Error: Skill 'non_existent' not found." in error

        # Test list_skill_files for directory traversal
        error = await list_skill_files("..")
        assert "Error: Access denied" in error

        error = await list_skill_files("../..")
        assert "Error: Access denied" in error

        # Test read_skill
        content = await read_skill(skill_name, "SKILL.md")
        assert content == "Test content"

        # Test read_skill for non-existent file
        error = await read_skill(skill_name, "missing.md")
        assert "Error: File 'missing.md' not found" in error

        # Test read_skill for non-existent skill
        error = await read_skill("missing_skill", "SKILL.md")
        # The exact message depends on implementation details, but it should mention not found
        assert "Error: Skill 'missing_skill' not found" in error

        # Test directory traversal
        # Try to read outside the skill directory
        # The path traversal is relative to the skill directory: mock_skills_dir / skill_name
        # We want to reach outside_file which is at tmp_path / outside.txt
        # Path from skill_dir to outside_file: ../../outside.txt
        traversal_path = f"../../{outside_file.name}"

        error = await read_skill(skill_name, traversal_path)
        assert "Error: Access denied" in error
