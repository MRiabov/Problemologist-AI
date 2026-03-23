from pathlib import Path

import pytest
import yaml


@pytest.mark.integration
@pytest.mark.integration_p0
def test_int_190_benchmark_coder_filesystem_scope_matches_workspace_contract():
    cfg = yaml.safe_load(Path("config/agents_config.yaml").read_text(encoding="utf-8"))
    role_cfg = cfg["agents"]["benchmark_coder"]["filesystem_permissions"]

    read_allow = set(role_cfg["read"]["allow"])
    write_allow = set(role_cfg["write"]["allow"])

    assert "**/*.py" in read_allow
    assert "**/*.py" in write_allow

    for required in {
        "skills/**",
        "utils/**",
        "plan.md",
        "todo.md",
        "journal.md",
        "benchmark_definition.yaml",
        "benchmark_assembly_definition.yaml",
        "script.py",
        "validation_results.json",
        "simulation_result.json",
        "scene.json",
        "renders/**",
    }:
        assert required in read_allow, f"benchmark_coder missing read scope {required}"

    for required in {"/", "."}:
        assert required in read_allow, (
            f"benchmark_coder missing workspace-root orientation scope {required}"
        )

    for forbidden in {"config/**", "shared", "shared/**"}:
        assert forbidden not in read_allow, (
            "benchmark_coder must stay within workspace artifacts and skills, "
            f"not repo-wide scope: found {forbidden}"
        )


@pytest.mark.integration
@pytest.mark.integration_p0
def test_int_190_unit_eval_allowlists_are_explicit_and_reviewer_driven():
    cfg = yaml.safe_load(Path("config/agents_config.yaml").read_text(encoding="utf-8"))
    agents = cfg["agents"]

    benchmark_coder_allowlist = set(
        agents["benchmark_coder"]["allowed_during_unit_eval"]
    )
    assert benchmark_coder_allowlist == {
        "benchmark_coder",
        "benchmark_reviewer",
    }
    assert "skill_agent" not in benchmark_coder_allowlist
    assert "git_agent" not in benchmark_coder_allowlist

    engineer_coder_allowlist = set(agents["engineer_coder"]["allowed_during_unit_eval"])
    assert engineer_coder_allowlist == {
        "engineer_coder",
        "engineer_execution_reviewer",
    }
    assert "skill_agent" not in engineer_coder_allowlist
    assert "git_agent" not in engineer_coder_allowlist

    benchmark_planner_allowlist = set(
        agents["benchmark_planner"]["allowed_during_unit_eval"]
    )
    assert benchmark_planner_allowlist == {
        "benchmark_planner",
        "benchmark_plan_reviewer",
    }

    engineer_planner_allowlist = set(
        agents["engineer_planner"]["allowed_during_unit_eval"]
    )
    assert engineer_planner_allowlist == {
        "engineer_planner",
        "engineer_plan_reviewer",
    }
    assert agents["engineer_planner"]["visual_inspection"]["required"] is True

    engineer_execution_reviewer_allowlist = set(
        agents["engineer_execution_reviewer"]["allowed_during_unit_eval"]
    )
    assert engineer_execution_reviewer_allowlist == {
        "engineer_coder",
        "engineer_execution_reviewer",
    }

    electronics_planner_allowlist = set(
        agents["electronics_planner"]["allowed_during_unit_eval"]
    )
    assert electronics_planner_allowlist == {
        "engineer_planner",
        "electronics_planner",
        "engineer_plan_reviewer",
    }

    git_agent_allowlist = set(agents["git_agent"]["allowed_during_unit_eval"])
    assert git_agent_allowlist == {"git_agent"}


@pytest.mark.integration
@pytest.mark.integration_p0
def test_int_190_agent_execution_timeouts_are_role_specific():
    cfg = yaml.safe_load(Path("config/agents_config.yaml").read_text(encoding="utf-8"))
    execution_agents = cfg["execution"]["agents"]

    assert execution_agents["engineer_coder"]["timeout_seconds"] == 1000
    assert execution_agents["benchmark_coder"]["timeout_seconds"] == 450
    assert execution_agents["engineer_plan_reviewer"]["timeout_seconds"] == 90
    assert execution_agents["benchmark_reviewer"]["timeout_seconds"] == 90
    assert execution_agents["engineer_execution_reviewer"]["timeout_seconds"] == 90
    assert execution_agents["electronics_reviewer"]["timeout_seconds"] == 90
