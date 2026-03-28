from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from build123d import Box, BuildPart

from evals.logic import runner
from evals.logic.codex_workspace import (
    build_codex_env,
    materialize_seed_workspace,
    prepare_codex_home,
    verify_planner_workspace,
)
from evals.logic.models import EvalDatasetItem
from evals.logic.seed_maintenance import refresh_plan_review_manifest_hashes
from shared.enums import AgentName, ManufacturingMethod, ReviewDecision
from shared.models.schemas import (
    DatasetCurationManifest,
    PartMetadata,
    PlannerSubmissionResult,
)
from worker_heavy.utils.build123d_rendering import render_preview_view

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


def _workspace_snapshot(workspace_dir: Path) -> dict[str, str]:
    snapshot: dict[str, str] = {}
    for path in sorted(p for p in workspace_dir.rglob("*") if p.is_file()):
        rel_path = path.relative_to(workspace_dir).as_posix()
        snapshot[rel_path] = hashlib.sha256(path.read_bytes()).hexdigest()
    return snapshot


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
    normalized_stdout = " ".join(completed.stdout.split())
    assert "--runner-backend" in normalized_stdout
    assert "--call-paid-api" in normalized_stdout
    assert "--level" in normalized_stdout
    assert re.search(
        r"default:\s*benchmark_planner\s*smoke-\s*test\s*run", completed.stdout
    )
    assert re.search(r"default:\s*1\s*smoke-\s*test\s*item", completed.stdout)
    assert re.search(r"default:\s*1\s*for\s*smoke-\s*test\s*runs", completed.stdout)
    assert "codex" in normalized_stdout


@pytest.mark.integration_p0
def test_run_evals_codex_exec_help_exposes_workspace_write_sandbox():
    completed = subprocess.run(
        ["codex", "exec", "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert "workspace-write" in completed.stdout
    assert "--sandbox" in completed.stdout


@pytest.mark.integration_p0
def test_run_evals_defaults_are_smoke_test_contract():
    from evals.logic.runner import _build_parser

    args = _build_parser().parse_args([])

    assert args.agent == "benchmark_planner"
    assert args.limit == 1
    assert args.concurrency == 1
    assert args.level is None


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_run_evals_codex_judge_does_not_launch_reviewers_without_flag(
    tmp_path, monkeypatch
):
    item = EvalDatasetItem(
        id="codex-judge-001",
        task="codex judge reviewer gate regression",
        complexity_level=0,
        expected_decision=ReviewDecision.APPROVED,
    )
    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()
    materialized = SimpleNamespace(
        workspace_dir=workspace_dir,
        prompt_path=workspace_dir / "prompt.md",
        prompt_text="prompt",
        copied_paths=(),
    )

    monkeypatch.setattr(runner, "SESSION_LOG_ROOT", tmp_path / "session-root")
    monkeypatch.setattr(
        runner, "_materialize_codex_workspace", lambda **_: materialized
    )
    monkeypatch.setattr(runner, "_launch_codex_exec", lambda *_, **__: 0)

    async def fake_verify_codex_workspace_for_agent(**_: object) -> SimpleNamespace:
        return SimpleNamespace(
            success=True,
            errors=[],
            details={},
            verification_name="codex-verify",
        )

    async def fake_record_hard_check_outcomes(**_: object) -> bool:
        return True

    async def fake_record_judge_outcomes(**_: object) -> None:
        return None

    monkeypatch.setattr(
        runner,
        "_verify_codex_workspace_for_agent",
        fake_verify_codex_workspace_for_agent,
    )
    monkeypatch.setattr(
        runner, "_record_hard_check_outcomes", fake_record_hard_check_outcomes
    )
    monkeypatch.setattr(runner, "_record_judge_outcomes", fake_record_judge_outcomes)
    monkeypatch.setattr(runner, "_write_eval_session_metadata", lambda **_: None)
    monkeypatch.setattr(
        runner,
        "_resolve_codex_home_root",
        lambda **_: tmp_path / "codex-home",
    )
    monkeypatch.setattr(
        runner,
        "_capture_latest_codex_session_artifacts",
        lambda **_: None,
    )

    reviewer_called = False

    async def fail_reviewer_chain_for_judge(**_: object) -> list[dict[str, object]]:
        nonlocal reviewer_called
        reviewer_called = True
        raise AssertionError(
            "reviewer chain should not run without run_reviewers_with_judge"
        )

    monkeypatch.setattr(
        runner,
        "_run_codex_reviewer_chain_for_judge",
        fail_reviewer_chain_for_judge,
    )

    stats = {AgentName.BENCHMARK_CODER: {"total": 0, "success": 0}}
    success = await runner._run_codex_eval(
        item=item,
        stats=stats,
        agent_name=AgentName.BENCHMARK_CODER,
        reward_agent_configs={},
        case_label="judge-gate",
        run_judge=True,
        run_reviewers_with_judge=False,
    )

    assert success is True
    assert reviewer_called is False
    assert stats[AgentName.BENCHMARK_CODER]["total"] == 1
    assert stats[AgentName.BENCHMARK_CODER]["success"] == 1


@pytest.mark.integration_p0
def test_run_evals_level_filter_parser_accepts_repeated_and_bracketed_values():
    from evals.logic.runner import _parse_level_filters

    assert _parse_level_filters(["0", "1,2", "[3,4]", "5 or 0"]) == {
        0,
        1,
        2,
        3,
        4,
        5,
    }


@pytest.mark.integration_p0
def test_run_evals_codex_env_uses_isolated_home_and_workspace_pythonpath(tmp_path):
    source_auth_path = tmp_path / "source-home" / ".codex" / "auth.json"
    source_auth_path.parent.mkdir(parents=True, exist_ok=True)
    source_auth_path.write_text(
        json.dumps(
            {
                "OPENAI_API_KEY": None,
                "tokens": {"account_id": "acct-1", "access_token": "token-1"},
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()
    codex_home_root = tmp_path / "codex-home"
    codex_home_dir = prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        source_auth_path=source_auth_path,
    )

    env = build_codex_env(
        task_id="task-1",
        workspace_dir=workspace_dir,
        codex_home_root=codex_home_root,
        session_id="session-1",
    )

    assert env["HOME"] == str(codex_home_root)
    assert env["CODEX_HOME"] == str(codex_home_dir)
    assert env["PYTHONPATH"].split(os.pathsep)[:2] == [
        str(workspace_dir.resolve()),
        str(ROOT),
    ]
    assert env["PROBLEMOLOGIST_REPO_ROOT"] == str(ROOT)
    assert env["PYTHON_BIN"] == str(ROOT / ".venv" / "bin" / "python")
    assert (
        json.loads((codex_home_dir / "auth.json").read_text(encoding="utf-8"))[
            "tokens"
        ]["account_id"]
        == "acct-1"
    )
    config_text = (codex_home_dir / "config.toml").read_text(encoding="utf-8")
    assert 'model = "gpt-5.4-mini"' in config_text
    assert 'model_reasoning_effort = "high"' in config_text
    assert "use_legacy_landlock = true" in config_text
    assert str(workspace_dir.resolve()) in config_text
    assert "mcp_servers" not in config_text


@pytest.mark.integration_p0
def test_run_evals_codex_env_supports_repo_root_imports(tmp_path):
    source_auth_path = tmp_path / "source-home" / ".codex" / "auth.json"
    source_auth_path.parent.mkdir(parents=True, exist_ok=True)
    source_auth_path.write_text(
        json.dumps(
            {
                "OPENAI_API_KEY": None,
                "tokens": {"account_id": "acct-1", "access_token": "token-1"},
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()
    (workspace_dir / "script.py").write_text(
        "from shared.models.schemas import PartMetadata\n"
        "print(PartMetadata(material_id='aluminum_6061', fixed=True).model_dump())\n",
        encoding="utf-8",
    )

    codex_home_root = tmp_path / "codex-home"
    prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        source_auth_path=source_auth_path,
    )

    env = build_codex_env(
        task_id="task-1",
        workspace_dir=workspace_dir,
        codex_home_root=codex_home_root,
        session_id="session-1",
    )

    completed = subprocess.run(
        [env["PYTHON_BIN"], "script.py"],
        cwd=workspace_dir,
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert "aluminum_6061" in completed.stdout


@pytest.mark.integration_p0
def test_run_evals_codex_vtk_preview_overrides_bad_display(tmp_path, monkeypatch):
    monkeypatch.setenv("DISPLAY", ":109")

    with BuildPart() as builder:
        Box(1, 1, 1)
    component = builder.part
    component.label = "display_box"
    component.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum_6061",
        fixed=True,
    )

    output_path = tmp_path / "renders" / "preview.png"
    rendered_path = render_preview_view(
        component,
        output_path=output_path,
        pitch=-35.0,
        yaw=45.0,
        workspace_root=tmp_path,
    )

    assert rendered_path == output_path
    assert output_path.exists(), output_path
    assert os.environ["DISPLAY"] != ":109"
    assert os.environ["DISPLAY"].startswith(":")
    assert output_path.stat().st_size > 0


@pytest.mark.integration_p0
def test_run_evals_codex_submit_helper_imports_workspace_script_from_cwd(
    tmp_path,
):
    item = _load_dataset_item(
        "dataset/data/seed/role_based/engineer_coder.json",
        "ec-001",
    )
    workspace_dir = tmp_path / "workspace"
    materialize_seed_workspace(
        item=item,
        agent_name=AgentName.ENGINEER_CODER,
        workspace_dir=workspace_dir,
    )

    env = os.environ.copy()
    env.update(
        {
            "PROBLEMOLOGIST_REPO_ROOT": str(ROOT),
            "PYTHON_BIN": str(ROOT / ".venv" / "bin" / "python"),
            "DISPLAY": ":109",
        }
    )
    completed = subprocess.run(
        ["bash", "scripts/submit_for_review.sh"],
        cwd=workspace_dir,
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )

    combined_output = "\n".join(
        part for part in (completed.stdout, completed.stderr) if part
    )
    assert "No module named 'script'" not in combined_output, combined_output
    assert '"stage": "load"' not in combined_output, combined_output
    assert (
        "validate_start" in combined_output
        or '"stage": "validation"' in combined_output
    )


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
        "expected_helper_scripts",
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
            ("scripts/submit_plan.sh",),
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
                "benchmark_assembly_definition.yaml",
                "assembly_definition.yaml",
                "scripts/submit_plan.sh",
                "scripts/submit_plan.py",
                "journal.md",
            ),
            ("scripts/submit_plan.sh",),
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
    expected_helper_scripts: tuple[str, ...],
):
    item = _load_dataset_item(seed_dataset, row_id)
    workspace_dir = tmp_path / agent_name.value / row_id
    materialized = materialize_seed_workspace(
        item=item,
        agent_name=agent_name,
        workspace_dir=workspace_dir,
    )
    mirror_workspace_dir = tmp_path / f"{agent_name.value}-mirror" / row_id
    mirror_materialized = materialize_seed_workspace(
        item=item,
        agent_name=agent_name,
        workspace_dir=mirror_workspace_dir,
    )

    assert materialized.helper_script_paths == list(expected_helper_scripts)
    assert mirror_materialized.helper_script_paths == list(expected_helper_scripts)
    assert "Workspace: current directory" in materialized.prompt_text
    assert "/workspace" not in materialized.prompt_text
    assert materialized.prompt_text == mirror_materialized.prompt_text
    assert materialized.copied_paths == mirror_materialized.copied_paths
    assert _workspace_snapshot(workspace_dir) == _workspace_snapshot(
        mirror_workspace_dir
    )
    for fragment in prompt_fragments:
        assert fragment in materialized.prompt_text
    assert not any(path.endswith("result.py") for path in materialized.copied_paths)
    for rel_path in expected_files:
        assert (workspace_dir / rel_path).exists(), rel_path

    shutil.rmtree(workspace_dir / ".manifests", ignore_errors=True)

    source_auth_path = tmp_path / "source-home" / ".codex" / "auth.json"
    source_auth_path.parent.mkdir(parents=True, exist_ok=True)
    source_auth_path.write_text(
        json.dumps(
            {
                "OPENAI_API_KEY": None,
                "tokens": {"account_id": "acct-1", "access_token": "token-1"},
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    codex_home_root = tmp_path / "codex-home"
    prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        source_auth_path=source_auth_path,
    )

    env = build_codex_env(
        task_id=item.id,
        workspace_dir=workspace_dir,
        codex_home_root=codex_home_root,
        session_id=f"INT-CODEX-{agent_name.value}-{row_id}",
    )
    assert "AGENT_NAME" not in env
    assert env["PROBLEMOLOGIST_REPO_ROOT"] == str(ROOT)
    assert str(ROOT) in env["PYTHONPATH"].split(os.pathsep)

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


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    (
        "seed_dataset",
        "row_id",
        "agent_name",
        "prompt_fragments",
        "expected_files",
        "expected_helper_scripts",
    ),
    [
        (
            "dataset/data/seed/role_based/benchmark_plan_reviewer.json",
            "bpr-012-gap-bridge-hidden-dof",
            AgentName.BENCHMARK_PLAN_REVIEWER,
            (
                "You are the Plan Reviewer.",
                "Inspect the planner artifacts",
                "reviews/",
                "bash scripts/submit_review.sh",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "scripts/submit_review.sh",
                "scripts/submit_review.py",
            ),
            ("scripts/submit_review.sh",),
        ),
        (
            "dataset/data/seed/role_based/benchmark_coder.json",
            "bc-011-sideways-ball",
            AgentName.BENCHMARK_CODER,
            (
                "You are the Coder for this workspace.",
                "Edit `script.py` and any supporting `*.py` files",
                "journal.md",
                "bash scripts/submit_for_review.sh",
                "utils.submission",
                "intermediate checks before",
                "result = build()",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "journal.md",
                "scripts/submit_for_review.sh",
                "scripts/submit_for_review.py",
            ),
            ("scripts/submit_for_review.sh",),
        ),
        (
            "dataset/data/seed/role_based/engineer_coder.json",
            "ec-001",
            AgentName.ENGINEER_CODER,
            (
                "You are the Coder for this workspace.",
                "Edit `script.py` and any supporting `*.py` files",
                "journal.md",
                "bash scripts/submit_for_review.sh",
                "utils.submission",
                "intermediate checks before",
                "result = build()",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "assembly_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "journal.md",
                "scripts/submit_for_review.sh",
                "scripts/submit_for_review.py",
            ),
            ("scripts/submit_for_review.sh",),
        ),
        (
            "dataset/data/seed/role_based/engineer_plan_reviewer.json",
            "epr-001-sideways-transfer",
            AgentName.ENGINEER_PLAN_REVIEWER,
            (
                "You are the Plan Reviewer.",
                "Inspect the planner artifacts",
                "reviews/",
                "bash scripts/submit_review.sh",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "assembly_definition.yaml",
                "scripts/submit_review.sh",
                "scripts/submit_review.py",
            ),
            ("scripts/submit_review.sh",),
        ),
        (
            "dataset/data/seed/role_based/benchmark_reviewer.json",
            "br-012-sideways-ball-infeasible-goal",
            AgentName.BENCHMARK_REVIEWER,
            (
                "You are the Execution Reviewer.",
                "Inspect the implementation, validation results, simulation result",
                "benchmark_assembly_definition.yaml",
                "reviews/",
                "bash scripts/submit_review.sh",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "script.py",
                "journal.md",
                "validation_results.json",
                "simulation_result.json",
                "scripts/submit_review.sh",
                "scripts/submit_review.py",
            ),
            ("scripts/submit_review.sh",),
        ),
        (
            "dataset/data/seed/role_based/engineer_execution_reviewer.json",
            "eer-002-gap-bridge",
            AgentName.ENGINEER_EXECUTION_REVIEWER,
            (
                "You are the Execution Reviewer.",
                "Inspect the implementation, validation results, simulation result",
                "reviews/",
                "bash scripts/submit_review.sh",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "assembly_definition.yaml",
                "script.py",
                "journal.md",
                "validation_results.json",
                "simulation_result.json",
                "scripts/submit_review.sh",
                "scripts/submit_review.py",
            ),
            ("scripts/submit_review.sh",),
        ),
        (
            "dataset/data/seed/role_based/electronics_reviewer.json",
            "erv-002-diverter-gate-review",
            AgentName.ELECTRONICS_REVIEWER,
            (
                "You are the Execution Reviewer.",
                "Inspect the implementation, validation results, simulation result",
                "reviews/",
                "bash scripts/submit_review.sh",
            ),
            (
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
                "assembly_definition.yaml",
                "script.py",
                "journal.md",
                "validation_results.json",
                "simulation_result.json",
                "scripts/submit_review.sh",
                "scripts/submit_review.py",
            ),
            ("scripts/submit_review.sh",),
        ),
    ],
)
async def test_codex_seed_workspace_materialization_is_role_specific_and_deterministic(
    tmp_path: Path,
    seed_dataset: str,
    row_id: str,
    agent_name: AgentName,
    prompt_fragments: tuple[str, ...],
    expected_files: tuple[str, ...],
    expected_helper_scripts: tuple[str, ...],
):
    """
    INT-034: reviewer evidence completeness.

    Verifies curated benchmark plan-reviewer, benchmark-coder, and benchmark-reviewer
    seed rows materialize into workspace-relative, deterministic local workspaces.
    """

    item = _load_dataset_item(seed_dataset, row_id)
    workspace_dir = tmp_path / agent_name.value / row_id
    mirror_workspace_dir = tmp_path / f"{agent_name.value}-mirror" / row_id

    materialized = materialize_seed_workspace(
        item=item,
        agent_name=agent_name,
        workspace_dir=workspace_dir,
    )
    mirror_materialized = materialize_seed_workspace(
        item=item,
        agent_name=agent_name,
        workspace_dir=mirror_workspace_dir,
    )

    assert materialized.helper_script_paths == list(expected_helper_scripts)
    assert mirror_materialized.helper_script_paths == list(expected_helper_scripts)
    assert materialized.prompt_text == mirror_materialized.prompt_text
    assert materialized.copied_paths == mirror_materialized.copied_paths
    assert _workspace_snapshot(workspace_dir) == _workspace_snapshot(
        mirror_workspace_dir
    )
    assert "Workspace: current directory" in materialized.prompt_text
    assert "/workspace" not in materialized.prompt_text
    for fragment in prompt_fragments:
        assert fragment in materialized.prompt_text
    for rel_path in expected_files:
        assert (workspace_dir / rel_path).exists(), rel_path


@pytest.mark.integration_p0
def test_validate_eval_seed_accepts_curated_rows_and_preserves_redundancy_metadata():
    """
    INT-114: benchmark planner explicit submission gate.

    Exercises the seed-validation CLI against representative curated rows and
    verifies the persisted curation manifests still expose deterministic
    redundancy provenance.
    """

    validation_cases = (
        ("benchmark_planner", "bp-001-forbid-zone"),
        ("benchmark_plan_reviewer", "bpr-012-gap-bridge-hidden-dof"),
        ("benchmark_coder", "bc-011-sideways-ball"),
        ("benchmark_reviewer", "br-012-sideways-ball-infeasible-goal"),
    )
    for agent_name, row_id in validation_cases:
        completed = subprocess.run(
            [
                sys.executable,
                "scripts/validate_eval_seed.py",
                "--skip-env-up",
                "--agent",
                agent_name,
                "--task-id",
                row_id,
                "--fail-fast",
                "--concurrency",
                "1",
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
            timeout=300,
        )

        assert completed.returncode == 0, completed.stderr
        assert "Validated 1 row(s): all passed." in completed.stdout, completed.stdout
        assert f"PASS {agent_name} {row_id}:" in completed.stdout, completed.stdout

    for manifest_path in (
        ROOT / "dataset/data/generated/component_seeded/v0.0.1/manifest.json",
        ROOT / "dataset/data/generated/workflow/v0.0.1/manifest.json",
    ):
        manifest = DatasetCurationManifest.model_validate_json(
            manifest_path.read_text(encoding="utf-8")
        )
        assert manifest.counts.accepted_after_pending_filter > 0
        assert manifest.counts.dedup_identity_groups_with_drops > 0
        assert manifest.counts.rejected > 0
        assert manifest.dropped_lineage, manifest_path
        assert manifest.rejected, manifest_path
        for lineage_key, dropped_episode_ids in manifest.dropped_lineage.items():
            assert lineage_key.strip()
            assert dropped_episode_ids == sorted(set(dropped_episode_ids))
        for rejected_row in manifest.rejected:
            assert rejected_row.reasons


@pytest.mark.integration_p0
def test_validate_eval_seed_can_filter_rows_by_complexity_level():
    completed = subprocess.run(
        [
            sys.executable,
            "scripts/validate_eval_seed.py",
            "--skip-env-up",
            "--agent",
            "cots_search",
            "--task-id",
            "cs-001-m3-bolt-match",
            "--level",
            "0",
            "--fail-fast",
            "--concurrency",
            "1",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
        timeout=300,
    )

    assert completed.returncode == 0, completed.stderr
    assert "Validated 1 row(s): all passed." in completed.stdout, completed.stdout
    assert "PASS cots_search cs-001-m3-bolt-match:" in completed.stdout, (
        completed.stdout
    )


@pytest.mark.integration_p0
def test_validate_eval_seed_errors_only_suppresses_pass_output():
    completed = subprocess.run(
        [
            sys.executable,
            "scripts/validate_eval_seed.py",
            "--skip-env-up",
            "--agent",
            "benchmark_planner",
            "--task-id",
            "bp-001-forbid-zone",
            "--fail-fast",
            "--concurrency",
            "1",
            "--errors-only",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
        timeout=300,
    )

    assert completed.returncode == 0, completed.stderr
    assert "PASS benchmark_planner bp-001-forbid-zone:" not in completed.stdout
    assert "Validated 1 row(s): all passed." not in completed.stdout


@pytest.mark.integration_p0
def test_refresh_plan_review_manifest_hashes_can_fix_drift(tmp_path: Path):
    artifact_dir = tmp_path / "seed_artifacts"
    artifact_dir.mkdir()

    payload_path = artifact_dir / "script.py"
    payload_path.write_text("print('hello world')\n", encoding="utf-8")

    manifest_path = artifact_dir / "benchmark_plan_review_manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "artifact_hashes": {
                    "script.py": "deadbeef",
                }
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    dry_run_updates = refresh_plan_review_manifest_hashes(artifact_dir, fix=False)
    assert dry_run_updates == [manifest_path]
    assert (
        json.loads(manifest_path.read_text(encoding="utf-8"))["artifact_hashes"][
            "script.py"
        ]
        == "deadbeef"
    )

    fixed_updates = refresh_plan_review_manifest_hashes(artifact_dir, fix=True)
    assert fixed_updates == [manifest_path]

    updated_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert (
        updated_manifest["artifact_hashes"]["script.py"]
        == hashlib.sha256(payload_path.read_bytes()).hexdigest()
    )
