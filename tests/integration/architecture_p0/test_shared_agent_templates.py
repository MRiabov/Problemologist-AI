from pathlib import Path

import pytest

from controller.agent.mock_scenarios import load_integration_mock_scenarios
from shared.agent_templates import load_common_template_files, load_template_text


@pytest.mark.integration_p0
def test_common_agent_templates_load_from_shared_templates():
    templates = load_common_template_files()
    assert set(templates) == {
        ".admin/clear_env.py",
        "journal.md",
        "manufacturing_config.yaml",
        "solution_script.py",
        "todo.md",
    }

    shared_root = Path("shared/agent_templates/common")
    assert templates[".admin/clear_env.py"] == (
        shared_root / ".admin/clear_env.py"
    ).read_text(encoding="utf-8")
    assert templates["solution_script.py"] == (
        shared_root / "solution_script.py"
    ).read_text(encoding="utf-8")
    assert templates["todo.md"] == (shared_root / "todo.md").read_text(encoding="utf-8")
    assert templates["journal.md"] == (shared_root / "journal.md").read_text(
        encoding="utf-8"
    )
    assert (
        load_template_text("common/solution_script.py")
        == templates["solution_script.py"]
    )


@pytest.mark.integration_p0
def test_template_file_expands_from_shared_agent_templates(tmp_path: Path):
    scenario_dir = tmp_path / "mock_responses"
    scenario_dir.mkdir()
    (scenario_dir / "INT-999.yaml").write_text(
        (
            "transcript:\n"
            "- node: engineer_coder\n"
            "  steps:\n"
            "  - tool_name: write_file\n"
            "    tool_args:\n"
            "      path: solution_script.py\n"
            "      overwrite: true\n"
            "      template_file: common/solution_script.py\n"
            "    expected_observation: success\n"
        ),
        encoding="utf-8",
    )

    scenarios = load_integration_mock_scenarios(scenario_dir)
    step = scenarios["INT-999"]["transcript"][0]["steps"][0]
    assert step["tool_args"]["content"] == (
        Path("shared/agent_templates/common/solution_script.py").read_text(
            encoding="utf-8"
        )
    )
