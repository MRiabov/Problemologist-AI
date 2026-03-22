from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from evals.logic.codex_workspace import (
    build_codex_env,
    materialize_seed_workspace,
    verify_planner_workspace,
)
from evals.logic.models import EvalDatasetItem
from shared.enums import AgentName
from shared.models.schemas import PlannerSubmissionResult

ROOT = Path(__file__).resolve().parents[3]


def _load_dataset_item(dataset_rel_path: str, row_id: str) -> EvalDatasetItem:
    json_path = ROOT / dataset_rel_path
    rows = json.loads(json_path.read_text(encoding="utf-8"))
    row = next(entry for entry in rows if entry["id"] == row_id)
    return EvalDatasetItem.model_validate(
        {**row, "seed_dataset": json_path.relative_to(ROOT)}
    )


def _load_submission_result(stdout: str) -> PlannerSubmissionResult:
    lines = stdout.splitlines()
    start_index = next(
        index for index, line in enumerate(lines) if line.lstrip().startswith("{")
    )
    json_text = "\n".join(lines[start_index:])
    return PlannerSubmissionResult.model_validate_json(json_text)


@pytest.mark.integration_p0
def test_run_evals_help_exposes_codex_backend():
    completed = subprocess.run(
        [sys.executable, "dataset/evals/run_evals.py", "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert "--runner-backend" in completed.stdout
    assert "--call-paid-api" in completed.stdout
    assert "codex" in completed.stdout


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    (
        "seed_dataset",
        "row_id",
        "agent_name",
        "expected_manifest",
        "prompt_fragments",
        "expected_files",
    ),
    [
        (
            "dataset/data/seed/role_based/benchmark_plan_reviewer.json",
            "bpr-001-raised-shelf",
            AgentName.BENCHMARK_PLANNER,
            ".manifests/benchmark_plan_review_manifest.json",
            (
                "Use workspace-relative paths only.",
                "bash scripts/submit_plan.sh",
                "benchmark_assembly_definition.yaml",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "scripts/submit_plan.sh",
                "scripts/submit_plan.py",
                "journal.md",
            ),
        ),
        (
            "dataset/data/seed/role_based/engineer_plan_reviewer.json",
            "epr-001-sideways-transfer",
            AgentName.ENGINEER_PLANNER,
            ".manifests/engineering_plan_review_manifest.json",
            (
                "Use workspace-relative paths only.",
                "bash scripts/submit_plan.sh",
                "assembly_definition.yaml",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "assembly_definition.yaml",
                "scripts/submit_plan.sh",
                "scripts/submit_plan.py",
                "journal.md",
            ),
        ),
    ],
)
async def test_codex_materialized_planner_workspace_submits(
    tmp_path: Path,
    seed_dataset: str,
    row_id: str,
    agent_name: AgentName,
    expected_manifest: str,
    prompt_fragments: tuple[str, ...],
    expected_files: tuple[str, ...],
):
    item = _load_dataset_item(seed_dataset, row_id)
    workspace_dir = tmp_path / agent_name.value / row_id
    materialized = materialize_seed_workspace(
        item=item,
        agent_name=agent_name,
        workspace_dir=workspace_dir,
    )

    assert materialized.helper_script_paths == ["scripts/submit_plan.sh"]
    assert "Workspace: current directory" in materialized.prompt_text
    assert "/workspace" not in materialized.prompt_text
    for fragment in prompt_fragments:
        assert fragment in materialized.prompt_text
    assert not any(path.endswith("result.py") for path in materialized.copied_paths)
    for rel_path in expected_files:
        assert (workspace_dir / rel_path).exists(), rel_path

    if agent_name == AgentName.BENCHMARK_PLANNER:
        assembly_path = workspace_dir / "benchmark_assembly_definition.yaml"
        assembly_payload = yaml.safe_load(assembly_path.read_text(encoding="utf-8"))
        assembly_payload["totals"]["estimated_unit_cost_usd"] = 71.5
        assembly_path.write_text(
            yaml.safe_dump(assembly_payload, sort_keys=False),
            encoding="utf-8",
        )

    shutil.rmtree(workspace_dir / ".manifests", ignore_errors=True)

    env = build_codex_env(
        task_id=item.id, session_id=f"INT-CODEX-{agent_name.value}-{row_id}"
    )
    assert "AGENT_NAME" not in env

    completed = subprocess.run(
        ["bash", "scripts/submit_plan.sh"],
        cwd=workspace_dir,
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    result = _load_submission_result(completed.stdout)
    assert result.ok is True
    assert result.status == "submitted"
    assert result.node_type == agent_name

    manifest_path = workspace_dir / expected_manifest
    assert manifest_path.exists(), manifest_path

    verification = await verify_planner_workspace(
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        session_id=env["SESSION_ID"],
    )
    assert verification.success, verification.errors
    assert verification.verification_name == "planner_workspace_contract"
