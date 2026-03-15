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
