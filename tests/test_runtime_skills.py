import unittest
from unittest.mock import patch, MagicMock
from pathlib import Path
import tempfile
import shutil

from src.environment.runtime import ToolRuntime

class TestRuntimeSkills(unittest.TestCase):
    def setUp(self):
        self.workspace_dir = tempfile.mkdtemp()
        self.skills_dir = tempfile.mkdtemp()
        self.runtime = ToolRuntime(workspace_dir=self.workspace_dir)

    def tearDown(self):
        shutil.rmtree(self.workspace_dir)
        shutil.rmtree(self.skills_dir)

    def test_write_skill(self):
        # Patch the Config object imported in runtime.py
        with patch('src.environment.runtime.Config') as mock_config:
            # Set SKILLS_DIR to be a real Path object
            mock_config.SKILLS_DIR = Path(self.skills_dir)

            skill_path = "docs/skills/test_skill.md"
            content = "# Test Skill"

            result = self.runtime.write_file(content, skill_path)

            self.assertIn("Successfully wrote", result)

            # Verify file exists in skills_dir
            created_file = Path(self.skills_dir) / "test_skill.md"
            self.assertTrue(created_file.exists())
            self.assertEqual(created_file.read_text(encoding="utf-8"), content)

            # Verify file does NOT exist in workspace under docs/skills
            workspace_file = Path(self.workspace_dir) / "docs/skills/test_skill.md"
            self.assertFalse(workspace_file.exists())

    def test_edit_skill(self):
        with patch('src.environment.runtime.Config') as mock_config:
            mock_config.SKILLS_DIR = Path(self.skills_dir)

            # Setup: Create file first
            skill_path = "docs/skills/test_skill_edit.md"
            content = "original content"
            (Path(self.skills_dir) / "test_skill_edit.md").write_text(content)

            # Edit
            result = self.runtime.edit_file(skill_path, "original", "new")

            self.assertIn("Successfully edited", result)

            # Verify
            edited_file = Path(self.skills_dir) / "test_skill_edit.md"
            self.assertEqual(edited_file.read_text(encoding="utf-8"), "new content")

    def test_write_workspace_file(self):
        # Verify normal file still goes to workspace
        with patch('src.environment.runtime.Config') as mock_config:
            mock_config.SKILLS_DIR = Path(self.skills_dir)

            path = "normal_file.txt"
            content = "normal content"

            result = self.runtime.write_file(content, path)

            self.assertIn("Successfully wrote", result)

            workspace_file = Path(self.workspace_dir) / path
            self.assertTrue(workspace_file.exists())

            # Should not be in skills
            self.assertFalse((Path(self.skills_dir) / path).exists())

    def test_edit_skill_path_traversal(self):
        with patch('src.environment.runtime.Config') as mock_config:
            mock_config.SKILLS_DIR = Path(self.skills_dir)

            # Attempt to traverse out of skills dir
            # path: docs/skills/../outside.txt
            # If we just removeprefix, we get "../outside.txt"
            # resolved against skills_dir -> skills_dir/../outside.txt -> temp_root/outside.txt
            # This should fail the check.

            skill_path = "docs/skills/../outside.txt"

            # Setup: Create file outside skills dir
            # Note: We need to be careful not to write to actual system root
            # self.skills_dir is a temp dir.

            result = self.runtime.edit_file(skill_path, "outside", "hacked")

            self.assertIn("Error:", result)
            self.assertIn("attempts to write outside", result)

if __name__ == '__main__':
    unittest.main()
