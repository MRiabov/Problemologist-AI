from __future__ import annotations

import contextlib
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field

from controller.agent.review_handover import (
    validate_plan_reviewer_handover,
    validate_reviewer_handover,
)
from evals.logic.models import EvalDatasetItem
from evals.logic.review_checks import (
    parse_review_decision_yaml,
    review_artifacts_complete_for_prefix,
    review_filename_candidates,
)
from evals.logic.workspace import resolve_seed_artifact_dir
from shared.agent_templates import (
    load_codex_template_files,
    load_common_template_files,
    load_role_template_files,
)
from shared.enums import AgentName
from shared.models.schemas import ReviewFrontmatter
from shared.workers.benchmark_definition_template import (
    ensure_benchmark_definition_yaml,
)
from shared.workers.filesystem.backend import FileInfo
from worker_heavy.utils.file_validation import (
    validate_node_output,
    validate_review_frontmatter,
)
from worker_heavy.workbenches.config import load_config, load_merged_config

ROOT = Path(__file__).resolve().parents[2]

_TEXT_SUFFIXES = {
    ".cfg",
    ".ini",
    ".json",
    ".md",
    ".py",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
    ".xml",
}
_BINARY_SUFFIXES = {
    ".gif",
    ".jpg",
    ".jpeg",
    ".mp4",
    ".pdf",
    ".png",
    ".pyc",
    ".pyo",
    ".step",
    ".stl",
    ".glb",
}


class MaterializedWorkspace(BaseModel):
    """Concrete on-disk workspace produced for a Codex run."""

    workspace_dir: Path
    prompt_path: Path
    prompt_text: str
    agent_name: AgentName
    task_id: str
    copied_paths: list[str] = Field(default_factory=list)
    helper_script_paths: list[str] = Field(default_factory=list)


class WorkspaceVerificationResult(BaseModel):
    """Outcome of local Codex workspace verification."""

    success: bool
    verification_name: str
    errors: list[str] = Field(default_factory=list)
    details: dict[str, Any] = Field(default_factory=dict)


@dataclass(slots=True)
class LocalWorkspaceClient:
    """Async filesystem adapter over a local Codex workspace."""

    root: Path
    session_id: str

    def _resolve(self, virtual_path: str) -> Path:
        normalized = virtual_path.strip()
        if normalized in {"/workspace", "workspace"}:
            normalized = "/"
        elif normalized.startswith("/workspace/"):
            normalized = "/" + normalized[len("/workspace/") :]
        elif normalized.startswith("workspace/"):
            normalized = normalized[len("workspace/") :]

        rel = normalized.lstrip("/")
        root = self.root.resolve()
        path = (root / rel).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise PermissionError(f"Path traversal attempted: {virtual_path}") from exc
        return path

    async def exists(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> bool:
        return self._resolve(path).exists()

    async def read_file(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> str:
        resolved = self._resolve(path)
        if not resolved.exists():
            return f"Error: File '{path}' not found."
        return resolved.read_text(encoding="utf-8")

    async def write_file(
        self,
        path: str,
        content: str,
        overwrite: bool = True,
        *,
        bypass_agent_permissions: bool = False,
    ) -> bool:
        resolved = self._resolve(path)
        if resolved.exists() and not overwrite:
            return False
        resolved.parent.mkdir(parents=True, exist_ok=True)
        resolved.write_text(content, encoding="utf-8")
        return True

    async def list_files(
        self, path: str = "/", *, bypass_agent_permissions: bool = False
    ) -> list[FileInfo]:
        resolved = self._resolve(path)
        if not resolved.exists():
            raise FileNotFoundError(f"Directory not found: {path}")
        if not resolved.is_dir():
            raise NotADirectoryError(f"Not a directory: {path}")

        entries: list[FileInfo] = []
        for child in sorted(resolved.iterdir(), key=lambda entry: entry.name):
            entries.append(
                FileInfo(
                    path="/" + child.relative_to(self.root).as_posix(),
                    name=child.name,
                    is_dir=child.is_dir(),
                    size=child.stat().st_size if child.is_file() else None,
                )
            )
        return entries

    async def aclose(self) -> None:
        return None


def is_planner_agent(agent_name: AgentName) -> bool:
    return agent_name in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.ENGINEER_PLANNER,
        AgentName.ELECTRONICS_PLANNER,
    }


def is_coder_agent(agent_name: AgentName) -> bool:
    return agent_name in {
        AgentName.BENCHMARK_CODER,
        AgentName.ENGINEER_CODER,
    }


def is_plan_reviewer_agent(agent_name: AgentName) -> bool:
    return agent_name in {
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.ENGINEER_PLAN_REVIEWER,
    }


def is_execution_reviewer_agent(agent_name: AgentName) -> bool:
    return agent_name in {
        AgentName.BENCHMARK_REVIEWER,
        AgentName.ENGINEER_EXECUTION_REVIEWER,
        AgentName.ELECTRONICS_REVIEWER,
    }


def is_reviewer_agent(agent_name: AgentName) -> bool:
    return is_plan_reviewer_agent(agent_name) or is_execution_reviewer_agent(agent_name)


def _workspace_files_to_validate(workspace_dir: Path) -> dict[str, str]:
    files: dict[str, str] = {}
    for path in sorted(p for p in workspace_dir.rglob("*") if p.is_file()):
        if any(part == "__pycache__" for part in path.parts):
            continue
        if path.suffix.lower() in _BINARY_SUFFIXES:
            continue
        if path.name == "prompt.md":
            continue
        if path.suffix.lower() not in _TEXT_SUFFIXES and path.name not in {
            "Dockerfile",
            "Makefile",
        }:
            continue

        with contextlib.suppress(UnicodeDecodeError):
            files[path.relative_to(workspace_dir).as_posix()] = path.read_text(
                encoding="utf-8"
            )
    return files


def _copy_tree(src_root: Path, dst_root: Path) -> list[str]:
    copied: list[str] = []
    for src_path in sorted(p for p in src_root.rglob("*") if p.is_file()):
        if "__pycache__" in src_path.parts or src_path.suffix in {".pyc", ".pyo"}:
            continue
        rel_path = src_path.relative_to(src_root)
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_path, dst_path)
        copied.append(rel_path.as_posix())
    return copied


def _write_template_files(dst_root: Path, template_files: dict[str, str]) -> list[str]:
    copied: list[str] = []
    for rel_path, content in sorted(template_files.items()):
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(content, encoding="utf-8")
        copied.append(rel_path)
    return copied


def _load_manufacturing_config(workspace_dir: Path):
    custom_config_path = workspace_dir / "manufacturing_config.yaml"
    if custom_config_path.exists():
        return load_merged_config(
            override_data=yaml.safe_load(custom_config_path.read_text()) or {}
        )
    return load_config()


def _planner_contract(agent_name: AgentName) -> tuple[str, AgentName]:
    if agent_name == AgentName.BENCHMARK_PLANNER:
        return (
            ".manifests/benchmark_plan_review_manifest.json",
            AgentName.BENCHMARK_PLAN_REVIEWER,
        )
    return (
        ".manifests/engineering_plan_review_manifest.json",
        AgentName.ENGINEER_PLAN_REVIEWER,
    )


def _review_contract(agent_name: AgentName) -> tuple[str, AgentName]:
    if agent_name == AgentName.BENCHMARK_CODER:
        return ".manifests/benchmark_review_manifest.json", AgentName.BENCHMARK_REVIEWER
    return (
        ".manifests/engineering_execution_review_manifest.json",
        AgentName.ENGINEER_EXECUTION_REVIEWER,
    )


def _review_prefix(agent_name: AgentName) -> str | None:
    return {
        AgentName.BENCHMARK_PLAN_REVIEWER: "benchmark-plan-review",
        AgentName.BENCHMARK_REVIEWER: "benchmark-execution-review",
        AgentName.ENGINEER_PLAN_REVIEWER: "engineering-plan-review",
        AgentName.ENGINEER_EXECUTION_REVIEWER: "engineering-execution-review",
        AgentName.ELECTRONICS_REVIEWER: "electronics-review",
    }.get(agent_name)


def build_codex_prompt(
    *,
    item: EvalDatasetItem,
    agent_name: AgentName,
) -> str:
    """Build a role-specific Codex prompt for the materialized workspace."""

    task = item.task.strip()
    workspace_note = (
        "Use workspace-relative paths only. Do not use absolute workspace "
        "root prefixes."
    )
    shared_note = (
        "The workspace already contains the starter files, role templates, and "
        "any copied seed artifacts. Treat `.manifests/` as system-owned and do "
        "not edit it directly."
    )

    if is_planner_agent(agent_name):
        if agent_name == AgentName.BENCHMARK_PLANNER:
            role_lines = [
                "You are the Benchmark Planner.",
                "Write and refine `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml`.",
                "Keep the benchmark definition internally consistent before submission.",
                "When the files are ready, run `bash scripts/submit_plan.sh` and keep iterating until it reports `ok=true` and `status=submitted`.",
                "Do not leave template placeholders in the submitted files.",
            ]
        else:
            role_lines = [
                "You are an Engineering Planner.",
                "Write and refine `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`.",
                "Treat `benchmark_assembly_definition.yaml` as benchmark-owned read-only handoff context copied into this workspace if it is present.",
                "When the files are ready, run `bash scripts/submit_plan.sh` and keep iterating until it reports `ok=true` and `status=submitted`.",
                "Do not leave template placeholders in the submitted files.",
            ]
    elif is_coder_agent(agent_name):
        role_lines = [
            "You are the Coder for this workspace.",
            "Edit `script.py` and any supporting `*.py` files until the implementation works.",
            "Use `python script.py` as the canonical execution path.",
            "Keep `todo.md` and `journal.md` in sync with the work you are actually doing.",
            "When the implementation is ready, make sure the normal validation, simulation, and review handoff artifacts exist.",
        ]
    elif is_plan_reviewer_agent(agent_name):
        role_lines = [
            "You are the Plan Reviewer.",
            "Inspect the planner artifacts and write the stage-specific review decision and comments files under `reviews/`.",
            "Use the latest planner handoff state; do not edit planner-owned source files.",
            "If the benchmark or engineer plan is invalid, reject it with concrete reasons in the review files.",
        ]
    elif is_execution_reviewer_agent(agent_name):
        role_lines = [
            "You are the Execution Reviewer.",
            "Inspect the implementation, validation results, simulation result, and stage-specific review files.",
            "Write the stage-specific review decision and comments files under `reviews/`.",
            "If the latest implementation is not reviewable, reject it with concrete reasons in the review files.",
        ]
    else:
        role_lines = [
            f"You are operating as `{agent_name.value}`.",
            "Use the workspace files directly and follow the seeded task instructions.",
        ]

    if item.seed_dataset is not None:
        dataset_note = f"Seed dataset: {item.seed_dataset}"
    else:
        dataset_note = "Seed dataset: not provided"

    prompt_lines = [
        "Workspace: current directory",
        f"Agent: {agent_name.value}",
        f"Task ID: {item.id}",
        dataset_note,
        "",
        "Task:",
        task,
        "",
        "Workspace contract:",
        f"- {workspace_note}",
        f"- {shared_note}",
    ]
    prompt_lines.extend(f"- {line}" for line in role_lines)
    return "\n".join(prompt_lines).rstrip() + "\n"


def materialize_seed_workspace(
    *,
    item: EvalDatasetItem,
    agent_name: AgentName,
    workspace_dir: Path,
) -> MaterializedWorkspace:
    """Materialize a seeded eval row into a local workspace."""

    workspace_dir = workspace_dir.expanduser().resolve()
    workspace_dir.mkdir(parents=True, exist_ok=True)

    copied_paths: list[str] = []
    copied_paths.extend(
        _write_template_files(workspace_dir, load_common_template_files())
    )

    if is_planner_agent(agent_name):
        copied_paths.extend(
            _write_template_files(workspace_dir, load_role_template_files(agent_name))
        )
        copied_paths.extend(
            _write_template_files(workspace_dir, load_codex_template_files())
        )

    artifact_dir = resolve_seed_artifact_dir(item, root=ROOT)
    if artifact_dir is not None:
        if not artifact_dir.exists():
            raise FileNotFoundError(
                f"Seed artifact directory not found: {artifact_dir}"
            )
        copied_paths.extend(_copy_tree(artifact_dir, workspace_dir))

    copied_paths.extend(
        _write_template_files(
            workspace_dir,
            {
                rel_path: content
                for rel_path, content in (item.seed_files or {}).items()
            },
        )
    )

    with contextlib.suppress(Exception):
        ensure_benchmark_definition_yaml(workspace_dir)

    prompt_text = build_codex_prompt(
        item=item,
        agent_name=agent_name,
    )
    prompt_path = workspace_dir / "prompt.md"
    prompt_path.write_text(prompt_text, encoding="utf-8")
    copied_paths.append("prompt.md")

    helper_script_paths = []
    if is_planner_agent(agent_name):
        helper_script_paths.append("scripts/submit_plan.sh")

    return MaterializedWorkspace(
        workspace_dir=workspace_dir,
        prompt_path=prompt_path,
        prompt_text=prompt_text,
        agent_name=agent_name,
        task_id=item.id,
        copied_paths=copied_paths,
        helper_script_paths=helper_script_paths,
    )


def build_codex_env(*, task_id: str, session_id: str | None = None) -> dict[str, str]:
    """Prepare a generic Codex subprocess environment for a local workspace run."""

    env = dict(os.environ)
    py_path = env.get("PYTHONPATH")
    env["PYTHONPATH"] = f"{ROOT}{os.pathsep}{py_path}" if py_path else str(ROOT)

    venv_bin = ROOT / ".venv" / "bin"
    if venv_bin.exists():
        current_path = env.get("PATH", "")
        env["PATH"] = (
            f"{venv_bin}{os.pathsep}{current_path}" if current_path else str(venv_bin)
        )
        env.setdefault("VIRTUAL_ENV", str(ROOT / ".venv"))
        env.setdefault("PYTHON_BIN", sys.executable)

    env.setdefault("CONTROLLER_URL", "http://localhost:18000")
    env.setdefault("WORKER_LIGHT_URL", "http://localhost:18001")
    env.setdefault("SESSION_ID", session_id or f"local-codex-{task_id}-{os.getpid()}")
    env.setdefault("IS_HEAVY_WORKER", "1")
    env.setdefault("PROBLEMOLOGIST_SCRIPT_IMPORT_MODE", "0")
    env.setdefault("COTS_DB_PATH", str(ROOT / "parts.db"))
    return env


def launch_codex_exec(
    workspace_dir: Path,
    prompt_text: str,
    *,
    task_id: str,
    session_id: str | None = None,
    yolo: bool = True,
) -> int:
    """Launch `codex exec` in a workspace and stream output to the terminal."""

    if shutil.which("codex") is None:
        raise FileNotFoundError("codex CLI was not found on PATH")

    cmd = [
        "codex",
        "exec",
        "-c",
        "shell_environment_policy.inherit=all",
        "--cd",
        str(workspace_dir),
        "--skip-git-repo-check",
    ]
    cmd.append("--yolo" if yolo else "--full-auto")
    cmd.append("-")
    print("launching: " + " ".join(cmd))
    completed = subprocess.run(
        cmd,
        input=prompt_text,
        text=True,
        env=build_codex_env(task_id=task_id, session_id=session_id),
        check=False,
    )
    return completed.returncode


def open_codex_ui(
    workspace_dir: Path,
    prompt_text: str,
    *,
    task_id: str,
    session_id: str | None = None,
    yolo: bool = True,
) -> int:
    """Open the interactive Codex UI with a workspace prompt."""

    if shutil.which("codex") is None:
        raise FileNotFoundError("codex CLI was not found on PATH")

    if not (workspace_dir / ".git").exists():
        subprocess.run(["git", "init", "-q", str(workspace_dir)], check=False)

    cmd = [
        "codex",
        "--cd",
        str(workspace_dir),
        "-c",
        "shell_environment_policy.inherit=all",
        "--no-alt-screen",
    ]
    cmd.insert(1, "--yolo" if yolo else "--full-auto")
    cmd.append(prompt_text)
    print("launching: " + " ".join(cmd))
    completed = subprocess.run(
        cmd,
        env=build_codex_env(task_id=task_id, session_id=session_id),
        check=False,
    )
    return completed.returncode


def _load_review_frontmatter_local(
    *,
    workspace_dir: Path,
    review_filename_prefix: str,
    session_id: str,
) -> tuple[ReviewFrontmatter | None, str | None]:
    reviews_dir = workspace_dir / "reviews"
    if not reviews_dir.exists():
        return None, "reviews/ directory not found"

    prefixes = review_filename_candidates(review_filename_prefix)
    yaml_patterns = [
        re.compile(rf"^{re.escape(prefix)}-decision-round-(\d+)\.yaml$")
        for prefix in prefixes
    ]
    md_patterns = [
        re.compile(rf"^{re.escape(prefix)}-(\d+)\.md$") for prefix in prefixes
    ]

    latest_yaml: tuple[int, Path] | None = None
    latest_md: tuple[int, Path] | None = None
    for entry in sorted(reviews_dir.iterdir(), key=lambda path: path.name):
        if entry.is_dir():
            continue
        for pattern in yaml_patterns:
            match = pattern.fullmatch(entry.name)
            if match is None:
                continue
            round_number = int(match.group(1))
            if latest_yaml is None or round_number > latest_yaml[0]:
                latest_yaml = (round_number, entry)
            break
        else:
            for pattern in md_patterns:
                match = pattern.fullmatch(entry.name)
                if match is None:
                    continue
                round_number = int(match.group(1))
                if latest_md is None or round_number > latest_md[0]:
                    latest_md = (round_number, entry)
                break

    if latest_yaml is not None:
        review_path = latest_yaml[1]
        try:
            content = review_path.read_text(encoding="utf-8")
        except Exception as exc:
            return None, f"failed to read {review_path}: {exc}"

        review_data, review_error = parse_review_decision_yaml(
            content,
            review_path=review_path.as_posix(),
        )
        if review_error is not None:
            return None, review_error
        if review_data is None:
            return None, f"missing review decision payload in {review_path}"
        return review_data, None

    if latest_md is not None:
        review_path = latest_md[1]
        try:
            content = review_path.read_text(encoding="utf-8")
        except Exception as exc:
            return None, f"failed to read {review_path}: {exc}"

        is_valid, review_data = validate_review_frontmatter(
            content,
            cad_agent_refused=True,
            session_id=session_id,
        )
        if not is_valid:
            details = (
                ", ".join(review_data)
                if isinstance(review_data, list)
                else str(review_data)
            )
            return None, f"invalid review frontmatter in {review_path}: {details}"
        if not isinstance(review_data, ReviewFrontmatter):
            return None, f"unexpected review frontmatter payload for {review_path}"
        return review_data, None

    return (
        None,
        "no persisted review artifact matching "
        f"reviews/{review_filename_prefix}-decision-round-<n>.yaml "
        f"or reviews/{review_filename_prefix}-<n>.md",
    )


async def verify_planner_workspace(
    *,
    workspace_dir: Path,
    agent_name: AgentName,
    session_id: str,
) -> WorkspaceVerificationResult:
    """Validate a planner workspace and its submitted handoff manifest."""

    artifacts = _workspace_files_to_validate(workspace_dir)
    manufacturing_config = _load_manufacturing_config(workspace_dir)
    is_valid, errors = validate_node_output(
        agent_name,
        artifacts,
        session_id=session_id,
        manufacturing_config=manufacturing_config,
    )
    if not is_valid:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="planner_workspace_contract",
            errors=errors,
        )

    manifest_path, expected_stage = _planner_contract(agent_name)
    local_client = LocalWorkspaceClient(root=workspace_dir, session_id=session_id)
    try:
        manifest_error = await validate_plan_reviewer_handover(
            local_client,
            manifest_path=manifest_path,
            expected_stage=expected_stage,
        )
    finally:
        await local_client.aclose()

    if manifest_error is not None:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="planner_workspace_manifest",
            errors=[manifest_error],
        )

    return WorkspaceVerificationResult(
        success=True,
        verification_name="planner_workspace_contract",
        details={
            "manifest_path": manifest_path,
            "expected_stage": expected_stage.value,
        },
    )


async def verify_coder_workspace(
    *,
    workspace_dir: Path,
    agent_name: AgentName,
    session_id: str,
) -> WorkspaceVerificationResult:
    """Validate a coder workspace and its execution-review handoff manifest."""

    artifacts = _workspace_files_to_validate(workspace_dir)
    manufacturing_config = _load_manufacturing_config(workspace_dir)
    is_valid, errors = validate_node_output(
        agent_name,
        artifacts,
        session_id=session_id,
        manufacturing_config=manufacturing_config,
    )
    if not is_valid:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="coder_workspace_contract",
            errors=errors,
        )

    manifest_path, expected_stage = _review_contract(agent_name)
    local_client = LocalWorkspaceClient(root=workspace_dir, session_id=session_id)
    try:
        manifest_error = await validate_reviewer_handover(
            local_client,
            manifest_path=manifest_path,
            expected_stage=expected_stage,
        )
    finally:
        await local_client.aclose()

    if manifest_error is not None:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="coder_workspace_manifest",
            errors=[manifest_error],
        )

    return WorkspaceVerificationResult(
        success=True,
        verification_name="coder_workspace_contract",
        details={
            "manifest_path": manifest_path,
            "expected_stage": expected_stage.value,
        },
    )


async def verify_reviewer_workspace(
    *,
    workspace_dir: Path,
    agent_name: AgentName,
    session_id: str,
    expected_decision: Any = None,
) -> WorkspaceVerificationResult:
    """Validate reviewer output files and the latest review decision."""

    review_prefix = _review_prefix(agent_name)
    if review_prefix is None:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="review_workspace_contract",
            errors=[f"Unsupported reviewer agent: {agent_name.value}"],
        )

    if is_plan_reviewer_agent(agent_name):
        manifest_path, expected_stage = _planner_contract(
            AgentName.BENCHMARK_PLANNER
            if agent_name == AgentName.BENCHMARK_PLAN_REVIEWER
            else AgentName.ENGINEER_PLANNER
        )
        local_client = LocalWorkspaceClient(root=workspace_dir, session_id=session_id)
        try:
            manifest_error = await validate_plan_reviewer_handover(
                local_client,
                manifest_path=manifest_path,
                expected_stage=expected_stage,
            )
        finally:
            await local_client.aclose()
        if manifest_error is not None:
            return WorkspaceVerificationResult(
                success=False,
                verification_name="review_workspace_manifest",
                errors=[manifest_error],
            )
    else:
        manifest_path = (
            ".manifests/benchmark_review_manifest.json"
            if agent_name == AgentName.BENCHMARK_REVIEWER
            else ".manifests/engineering_execution_review_manifest.json"
            if agent_name == AgentName.ENGINEER_EXECUTION_REVIEWER
            else ".manifests/electronics_review_manifest.json"
        )
        expected_stage = (
            AgentName.BENCHMARK_REVIEWER
            if agent_name == AgentName.BENCHMARK_REVIEWER
            else AgentName.ENGINEER_EXECUTION_REVIEWER
            if agent_name == AgentName.ENGINEER_EXECUTION_REVIEWER
            else AgentName.ELECTRONICS_REVIEWER
        )
        local_client = LocalWorkspaceClient(root=workspace_dir, session_id=session_id)
        try:
            manifest_error = await validate_reviewer_handover(
                local_client,
                manifest_path=manifest_path,
                expected_stage=expected_stage,
            )
        finally:
            await local_client.aclose()
        if manifest_error is not None:
            return WorkspaceVerificationResult(
                success=False,
                verification_name="review_workspace_manifest",
                errors=[manifest_error],
            )

    local_client = LocalWorkspaceClient(root=workspace_dir, session_id=session_id)
    try:
        review_complete = await review_artifacts_complete_for_prefix(
            worker=local_client,
            review_filename_prefix=review_prefix,
        )
    finally:
        await local_client.aclose()
    if not review_complete:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="review_workspace_artifacts",
            errors=[
                f"review artifacts incomplete for prefix {review_prefix}",
            ],
        )

    review_data, review_error = _load_review_frontmatter_local(
        workspace_dir=workspace_dir,
        review_filename_prefix=review_prefix,
        session_id=session_id,
    )
    if review_error is not None:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="review_workspace_artifacts",
            errors=[review_error],
        )

    if expected_decision is not None and review_data is not None:
        observed_decision = getattr(review_data, "decision", None)
        if observed_decision != expected_decision:
            return WorkspaceVerificationResult(
                success=False,
                verification_name="review_workspace_decision",
                errors=[
                    "expected review decision "
                    f"{getattr(expected_decision, 'value', expected_decision)}, "
                    f"got {getattr(observed_decision, 'value', observed_decision)}"
                ],
            )

    return WorkspaceVerificationResult(
        success=True,
        verification_name="review_workspace_contract",
        details={
            "review_prefix": review_prefix,
            "expected_decision": (
                getattr(expected_decision, "value", expected_decision)
                if expected_decision is not None
                else None
            ),
        },
    )


async def verify_workspace_for_agent(
    *,
    workspace_dir: Path,
    agent_name: AgentName,
    session_id: str,
    expected_decision: Any = None,
) -> WorkspaceVerificationResult:
    """Dispatch local verification to the appropriate role-specific checks."""

    if is_planner_agent(agent_name):
        return await verify_planner_workspace(
            workspace_dir=workspace_dir,
            agent_name=agent_name,
            session_id=session_id,
        )
    if is_coder_agent(agent_name):
        return await verify_coder_workspace(
            workspace_dir=workspace_dir,
            agent_name=agent_name,
            session_id=session_id,
        )
    if is_reviewer_agent(agent_name):
        return await verify_reviewer_workspace(
            workspace_dir=workspace_dir,
            agent_name=agent_name,
            session_id=session_id,
            expected_decision=expected_decision,
        )

    return WorkspaceVerificationResult(
        success=False,
        verification_name="unsupported_codex_role",
        errors=[f"Unsupported Codex role: {agent_name.value}"],
    )
