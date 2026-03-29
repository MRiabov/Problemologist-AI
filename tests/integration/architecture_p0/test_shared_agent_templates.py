from pathlib import Path

import pytest

from controller.agent.mock_scenarios import load_integration_mock_scenarios
from shared.agent_templates import load_common_template_files, load_template_text


@pytest.mark.integration_p0
def test_common_agent_templates_match_worker_light_starters():
    templates = load_common_template_files()
    assert set(templates) == {
        "journal.md",
        "manufacturing_config.yaml",
        "script.py",
        "todo.md",
    }

    worker_light_root = Path("worker_light/agent_files")
    assert templates["script.py"] == (worker_light_root / "script.py").read_text(
        encoding="utf-8"
    )
    assert templates["todo.md"] == (worker_light_root / "todo.md").read_text(
        encoding="utf-8"
    )
    assert templates["journal.md"] == (worker_light_root / "journal.md").read_text(
        encoding="utf-8"
    )
    assert templates["manufacturing_config.yaml"] == (
        Path("shared/agent_templates/common/manufacturing_config.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert load_template_text("common/script.py") == templates["script.py"]
    assert (
        load_template_text("common/manufacturing_config.yaml")
        == templates["manufacturing_config.yaml"]
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
            "      path: script.py\n"
            "      overwrite: true\n"
            "      template_file: common/script.py\n"
            "    expected_observation: success\n"
        ),
        encoding="utf-8",
    )

    scenarios = load_integration_mock_scenarios(scenario_dir)
    step = scenarios["INT-999"]["transcript"][0]["steps"][0]
    assert step["tool_args"]["content"] == (
        Path("worker_light/agent_files/script.py").read_text(encoding="utf-8")
    )
