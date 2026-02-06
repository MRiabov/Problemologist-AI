import os
from pathlib import Path

def test_skills_presence():
    """Verify that the required skills are present in the worker's source tree."""

    # Define the base path relative to the repo root (assuming tests are run from root)
    base_path = Path("src/worker/skills")

    # Required skills as per prompts.yaml
    required_skills = [
        "build123d_cad_drafting_skill",
        "manufacturing-knowledge"
    ]

    # Verify base directory exists
    assert base_path.exists(), f"Skills directory {base_path} does not exist"
    assert base_path.is_dir(), f"{base_path} is not a directory"

    for skill in required_skills:
        skill_dir = base_path / skill
        skill_file = skill_dir / "SKILL.md"

        # Check directory exists
        assert skill_dir.exists(), f"Skill directory {skill} missing in {base_path}"
        assert skill_dir.is_dir(), f"{skill_dir} is not a directory"

        # Check SKILL.md exists
        assert skill_file.exists(), f"SKILL.md missing for {skill}"
        assert skill_file.is_file(), f"{skill_file} is not a file"

        # Check content is not empty
        content = skill_file.read_text()
        assert len(content) > 0, f"SKILL.md for {skill} is empty"
        assert f"name: {skill}" in content, f"SKILL.md for {skill} does not contain correct name"

if __name__ == "__main__":
    test_skills_presence()
    print("All skills verification passed!")
