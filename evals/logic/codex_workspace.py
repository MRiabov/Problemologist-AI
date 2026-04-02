from __future__ import annotations

import contextlib
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, Field

from controller.agent.prompt_manager import PromptBackendFamily, PromptManager
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
    load_template_repo_files,
)
from shared.agents.config import DraftingMode, load_agents_config
from shared.enums import AgentName
from shared.git_utils import repo_revision
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    CostTotals,
    DraftingCallout,
    DraftingDimension,
    DraftingNote,
    DraftingSheet,
    DraftingView,
    PartConfig,
    ReviewComments,
    ReviewFrontmatter,
)
from shared.runtime.headless import load_headless_opengl_config
from shared.workers.filesystem.backend import FileInfo
from worker_heavy.utils.file_validation import (
    validate_node_output,
    validate_review_frontmatter,
)
from worker_heavy.workbenches.config import load_config, load_merged_config

ROOT = Path(__file__).resolve().parents[2]
_CODEX_RUNTIME_ROOT_NAME = "codex-runtime"
SKILL_SOURCE_ROOT = ROOT / "skills"
CODEX_SKILL_TREE_ROOT = Path(".agents") / "skills"

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
_SKIP_COPY_DIR_NAMES = {
    ".git",
    "__pycache__",
}
_REASONING_EFFORT_UNSET = object()
_ENGINEER_DRAFTING_TARGETS = {
    AgentName.ENGINEER_PLANNER,
    AgentName.ENGINEER_PLAN_REVIEWER,
    AgentName.ENGINEER_CODER,
    AgentName.ENGINEER_EXECUTION_REVIEWER,
}

_ENGINEER_BENCHMARK_CONTEXT_TARGETS = {
    AgentName.ENGINEER_PLANNER,
    AgentName.ENGINEER_CODER,
}

_BENCHMARK_DRAFTING_TARGETS = {
    AgentName.BENCHMARK_PLANNER,
    AgentName.BENCHMARK_PLAN_REVIEWER,
    AgentName.BENCHMARK_CODER,
    AgentName.BENCHMARK_REVIEWER,
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


class CodexExecRunResult(BaseModel):
    """Outcome of a Codex CLI invocation."""

    command: list[str] = Field(default_factory=list)
    return_code: int | None = None
    timed_out: bool = False
    timeout_seconds: float | None = None
    session_id: str | None = None
    output_last_message_path: Path | None = None


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

    async def read_file_optional(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> str | None:
        resolved = self._resolve(path)
        if not resolved.exists():
            return None
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
        relative_parts = path.relative_to(workspace_dir).parts
        if relative_parts and (
            relative_parts[0] == "skills" or relative_parts[:2] == (".agents", "skills")
        ):
            continue
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
        if any(
            part in _SKIP_COPY_DIR_NAMES for part in src_path.parts
        ) or src_path.suffix in {".pyc", ".pyo"}:
            continue
        rel_path = src_path.relative_to(src_root)
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_path, dst_path)
        copied.append(rel_path.as_posix())
    return copied


def _copy_skills_tree(dst_root: Path) -> list[str]:
    if not SKILL_SOURCE_ROOT.exists():
        raise FileNotFoundError(f"Skill repository not found: {SKILL_SOURCE_ROOT}")
    return [
        f"{CODEX_SKILL_TREE_ROOT.as_posix()}/{path}"
        for path in _copy_tree(SKILL_SOURCE_ROOT, dst_root / CODEX_SKILL_TREE_ROOT)
    ]


def _write_missing_template_files(
    dst_root: Path, template_files: dict[str, str]
) -> list[str]:
    copied: list[str] = []
    for rel_path, content in sorted(template_files.items()):
        dst_path = dst_root / rel_path
        if dst_path.exists():
            continue
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(content, encoding="utf-8")
        copied.append(rel_path)
    return copied


def _drafting_mode_active(mode: DraftingMode) -> bool:
    return mode in (DraftingMode.DRAFTING, DraftingMode.DRAWING)


def _engineering_drafting_mode_active() -> bool:
    try:
        return _drafting_mode_active(
            load_agents_config().get_drafting_mode(AgentName.ENGINEER_PLANNER)
        )
    except Exception:
        return False


def _benchmark_drafting_mode_active() -> bool:
    try:
        return _drafting_mode_active(
            load_agents_config().get_drafting_mode(AgentName.BENCHMARK_PLANNER)
        )
    except Exception:
        return False


def _collect_assembly_targets(assembly_definition: AssemblyDefinition) -> list[str]:
    targets: list[str] = []

    def _add(value: object) -> None:
        text = str(value).strip()
        if text and text not in targets:
            targets.append(text)

    for part in assembly_definition.manufactured_parts:
        _add(part.part_name)
        _add(part.part_id)
    for part in assembly_definition.cots_parts:
        _add(part.part_id)
    for item in assembly_definition.final_assembly:
        if isinstance(item, PartConfig):
            _add(item.name)
            continue
        _add(item.subassembly_id)
        for part in item.parts:
            _add(part.name)
    if assembly_definition.electronics:
        for component in assembly_definition.electronics.components:
            if component.assembly_part_ref:
                _add(component.assembly_part_ref)

    return targets


def _starter_drafting_sheet(target_name: str) -> DraftingSheet:
    return DraftingSheet(
        sheet_id="sheet-1",
        title="Starter Drafting Package",
        views=[
            DraftingView(
                view_id="front",
                target=target_name,
                projection="front",
                scale=1.0,
                datums=["A"],
                dimensions=[
                    DraftingDimension(
                        dimension_id="starter_width",
                        kind="linear",
                        target=target_name,
                        value=1.0,
                        binding=True,
                        note="Starter placeholder dimension.",
                    )
                ],
                callouts=[
                    DraftingCallout(
                        callout_id="1",
                        label="Starter assembly",
                        target=target_name,
                    )
                ],
                notes=[
                    DraftingNote(
                        note_id="n1",
                        text="Starter drafting placeholder for the seeded workspace.",
                        critical=False,
                    )
                ],
            )
        ],
    )


def _load_benchmark_caps(workspace_dir: Path) -> tuple[float, float]:
    benchmark_path = workspace_dir / "benchmark_definition.yaml"
    default_unit_cost = 100.0
    default_weight = 1000.0
    if not benchmark_path.exists():
        return default_unit_cost, default_weight
    try:
        data = yaml.safe_load(benchmark_path.read_text(encoding="utf-8")) or {}
        benchmark = BenchmarkDefinition.model_validate(data)
    except Exception:
        return default_unit_cost, default_weight

    max_unit_cost = (
        benchmark.constraints.max_unit_cost
        if benchmark.constraints.max_unit_cost is not None
        else default_unit_cost
    )
    max_weight_g = (
        benchmark.constraints.max_weight_g
        if benchmark.constraints.max_weight_g is not None
        else default_weight
    )
    return float(max_unit_cost), float(max_weight_g)


def _starter_engineer_assembly(workspace_dir: Path) -> AssemblyDefinition:
    benchmark_max_unit_cost_usd, benchmark_max_weight_g = _load_benchmark_caps(
        workspace_dir
    )
    planner_target_max_unit_cost_usd = max(1.0, benchmark_max_unit_cost_usd * 0.5)
    planner_target_max_weight_g = max(1.0, benchmark_max_weight_g * 0.5)
    target_name = "starter_assembly"
    return AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=benchmark_max_unit_cost_usd,
            benchmark_max_weight_g=benchmark_max_weight_g,
            planner_target_max_unit_cost_usd=planner_target_max_unit_cost_usd,
            planner_target_max_weight_g=planner_target_max_weight_g,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[PartConfig(name=target_name, config=AssemblyPartConfig())],
        totals=CostTotals(
            estimated_unit_cost_usd=0.0,
            estimated_weight_g=0.0,
            estimate_confidence="high",
        ),
        drafting=_starter_drafting_sheet(target_name),
    )


def _starter_benchmark_assembly(workspace_dir: Path) -> AssemblyDefinition:
    benchmark_max_unit_cost_usd, benchmark_max_weight_g = _load_benchmark_caps(
        workspace_dir
    )
    planner_target_max_unit_cost_usd = max(1.0, benchmark_max_unit_cost_usd * 0.5)
    planner_target_max_weight_g = max(1.0, benchmark_max_weight_g * 0.5)
    target_name = "starter_assembly"
    return AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=benchmark_max_unit_cost_usd,
            benchmark_max_weight_g=benchmark_max_weight_g,
            planner_target_max_unit_cost_usd=planner_target_max_unit_cost_usd,
            planner_target_max_weight_g=planner_target_max_weight_g,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[PartConfig(name=target_name, config=AssemblyPartConfig())],
        totals=CostTotals(
            estimated_unit_cost_usd=0.0,
            estimated_weight_g=0.0,
            estimate_confidence="high",
        ),
        drafting=_starter_drafting_sheet(target_name),
    )


def _ensure_engineer_drafting_contract(workspace_dir: Path) -> list[str]:
    assembly_path = workspace_dir / "assembly_definition.yaml"
    if not assembly_path.exists():
        return []

    raw_content = assembly_path.read_text(encoding="utf-8")
    try:
        parsed = yaml.safe_load(raw_content) or {}
        assembly = AssemblyDefinition.model_validate(parsed)
    except Exception:
        starter = _starter_engineer_assembly(workspace_dir)
        assembly_path.write_text(
            yaml.safe_dump(
                starter.model_dump(mode="json", by_alias=True, exclude_none=True),
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return ["assembly_definition.yaml"]

    if assembly.drafting is not None:
        return []

    targets = _collect_assembly_targets(assembly)
    if not targets:
        starter = _starter_engineer_assembly(workspace_dir)
        assembly_path.write_text(
            yaml.safe_dump(
                starter.model_dump(mode="json", by_alias=True, exclude_none=True),
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return ["assembly_definition.yaml"]

    updated = assembly.model_copy(deep=True)
    updated.drafting = _starter_drafting_sheet(targets[0])
    assembly_path.write_text(
        yaml.safe_dump(
            updated.model_dump(mode="json", by_alias=True, exclude_none=True),
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return ["assembly_definition.yaml"]


def _ensure_benchmark_drafting_contract(workspace_dir: Path) -> list[str]:
    assembly_path = workspace_dir / "benchmark_assembly_definition.yaml"
    if not assembly_path.exists():
        return []

    raw_content = assembly_path.read_text(encoding="utf-8")
    try:
        parsed = yaml.safe_load(raw_content) or {}
        assembly = AssemblyDefinition.model_validate(parsed)
    except Exception:
        starter = _starter_benchmark_assembly(workspace_dir)
        assembly_path.write_text(
            yaml.safe_dump(
                starter.model_dump(mode="json", by_alias=True, exclude_none=True),
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return ["benchmark_assembly_definition.yaml"]

    if assembly.drafting is not None:
        return []

    targets = _collect_assembly_targets(assembly)
    if not targets:
        starter = _starter_benchmark_assembly(workspace_dir)
        assembly_path.write_text(
            yaml.safe_dump(
                starter.model_dump(mode="json", by_alias=True, exclude_none=True),
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        return ["benchmark_assembly_definition.yaml"]

    updated = assembly.model_copy(deep=True)
    updated.drafting = _starter_drafting_sheet(targets[0])
    assembly_path.write_text(
        yaml.safe_dump(
            updated.model_dump(mode="json", by_alias=True, exclude_none=True),
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return ["benchmark_assembly_definition.yaml"]


def copy_workspace_contents(
    source_root: Path,
    dst_root: Path,
    *,
    exclude_rel_paths: set[str] | None = None,
) -> list[str]:
    """Copy a workspace snapshot into another workspace without overwriting exclusions."""

    source_root = source_root.expanduser().resolve()
    dst_root = dst_root.expanduser().resolve()
    exclude_rel_paths = {
        Path(path).as_posix().lstrip("/") for path in (exclude_rel_paths or set())
    }

    copied: list[str] = []
    for src_path in sorted(p for p in source_root.rglob("*") if p.is_file()):
        rel_path = src_path.relative_to(source_root).as_posix()
        if any(
            rel_path == excluded or rel_path.startswith(f"{excluded}/")
            for excluded in exclude_rel_paths
        ):
            continue
        if any(part in _SKIP_COPY_DIR_NAMES for part in src_path.parts):
            continue
        if src_path.suffix in {".pyc", ".pyo"}:
            continue
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_path, dst_path)
        copied.append(rel_path)
    return copied


def _write_template_files(dst_root: Path, template_files: dict[str, str]) -> list[str]:
    copied: list[str] = []
    for rel_path, content in sorted(template_files.items()):
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(content, encoding="utf-8")
        copied.append(rel_path)
    return copied


def resolve_codex_home_root(
    *,
    task_id: str,
    session_id: str | None = None,
    runtime_root: Path | None = None,
) -> Path:
    """Return the isolated HOME directory used for a Codex run."""

    runtime_root = (runtime_root or ROOT) / _CODEX_RUNTIME_ROOT_NAME
    session_key = session_id or f"local-codex-{task_id}-{os.getpid()}"
    return runtime_root / "homes" / session_key


def prepare_codex_home(
    *,
    codex_home_root: Path,
    workspace_dir: Path,
    source_auth_path: Path | None = None,
    agent_name: AgentName | None = None,
    reasoning_effort: Literal["low", "medium", "high", "xhigh"]
    | None
    | object = _REASONING_EFFORT_UNSET,
) -> Path:
    """Seed an isolated Codex home with the active auth bundle."""

    source_auth_path = source_auth_path or (Path.home() / ".codex" / "auth.json")
    if not source_auth_path.exists():
        raise FileNotFoundError(f"Codex auth file not found: {source_auth_path}")

    codex_home_dir = codex_home_root / ".codex"
    codex_home_dir.mkdir(parents=True, exist_ok=True)
    # Give each isolated run its own writable cache/config roots so library
    # cache initialization does not spill across parallel sessions.
    (codex_home_root / ".cache").mkdir(parents=True, exist_ok=True)
    (codex_home_root / ".config").mkdir(parents=True, exist_ok=True)
    (codex_home_root / ".tmp").mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_auth_path, codex_home_dir / "auth.json")
    workspace_path = workspace_dir.expanduser().resolve()
    config = (
        load_agents_config()
        if agent_name is not None or reasoning_effort is not _REASONING_EFFORT_UNSET
        else None
    )
    if config is not None and not config.llm.reasoning_effort_enabled:
        resolved_reasoning_effort = None
    elif reasoning_effort is _REASONING_EFFORT_UNSET:
        resolved_reasoning_effort = (
            config.get_reasoning_effort(agent_name)
            if config is not None and agent_name is not None
            else "high"
        )
    else:
        resolved_reasoning_effort = reasoning_effort
    config_path = codex_home_dir / "config.toml"
    config_lines = [
        'model = "gpt-5.4-mini"',
        'approvals_reviewer = "user"',
        "",
        "[features]",
        "use_legacy_landlock = true",
        "",
        f'[projects."{workspace_path.as_posix()}"]',
        'trust_level = "trusted"',
        "respect_gitignore = false",
        "",
    ]
    if resolved_reasoning_effort is not None:
        config_lines.insert(
            1, f'model_reasoning_effort = "{resolved_reasoning_effort}"'
        )
    config_path.write_text("\n".join(config_lines), encoding="utf-8")
    return codex_home_dir


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


def _build_codex_runtime_context(
    *, item: EvalDatasetItem, agent_name: AgentName
) -> str:
    task = item.task.strip()
    if item.seed_dataset is not None:
        dataset_note = f"Seed dataset: {item.seed_dataset}"
    else:
        dataset_note = "Seed dataset: not provided"

    runtime_context_lines = [
        "Workspace: current directory",
        f"Agent: {agent_name.value}",
        f"Task ID: {item.id}",
        dataset_note,
        "",
        "Task:",
        task,
        "",
        "Workspace contract:",
        "- Use workspace-relative paths only.",
        "- The workspace already contains the starter files, role templates, and any copied seed artifacts.",
        "- Treat `.manifests/` as system-owned and do not edit it directly.",
        "- If you need a clean retry, run `python .admin/clear_env.py` to restore the seeded workspace in place.",
    ]
    return "\n".join(runtime_context_lines)


def build_codex_prompt(
    *,
    item: EvalDatasetItem,
    agent_name: AgentName,
) -> str:
    """Build a role-specific Codex prompt for the materialized workspace."""

    runtime_context = _build_codex_runtime_context(item=item, agent_name=agent_name)
    prompt_manager = PromptManager()
    return prompt_manager.render(
        agent_name,
        backend_family=PromptBackendFamily.CLI_BASED,
        runtime_context=runtime_context,
    )


def _run_codex_exec_command(
    *,
    workspace_dir: Path,
    prompt_text: str,
    task_id: str,
    agent_name: AgentName | None = None,
    session_id: str | None = None,
    runtime_root: Path | None = None,
    yolo: bool = True,
    timeout_seconds: float | None = None,
    resume_session_id: str | None = None,
    output_last_message_path: Path | None = None,
    reasoning_effort: Literal["low", "medium", "high", "xhigh"]
    | None
    | object = _REASONING_EFFORT_UNSET,
) -> CodexExecRunResult:
    codex_home_root = resolve_codex_home_root(
        task_id=task_id,
        session_id=session_id,
        runtime_root=runtime_root,
    )
    prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        reasoning_effort=reasoning_effort,
    )

    cmd = ["codex", "exec"]
    if resume_session_id is not None:
        cmd.extend(["resume", resume_session_id])
    cmd.extend(_codex_execution_flags(yolo=yolo))
    if output_last_message_path is not None:
        output_last_message_path.parent.mkdir(parents=True, exist_ok=True)
        cmd.extend(["--output-last-message", str(output_last_message_path)])
    cmd.extend(
        [
            "-c",
            "shell_environment_policy.inherit=all",
            "--cd",
            str(workspace_dir),
            "--skip-git-repo-check",
            "-",
        ]
    )
    print("launching: " + " ".join(cmd))

    result = CodexExecRunResult(
        command=cmd,
        timeout_seconds=timeout_seconds,
        session_id=session_id,
        output_last_message_path=output_last_message_path,
    )
    try:
        completed = subprocess.run(
            cmd,
            input=prompt_text,
            text=True,
            env=build_codex_env(
                task_id=task_id,
                workspace_dir=workspace_dir,
                codex_home_root=codex_home_root,
                session_id=session_id,
                agent_name=agent_name,
            ),
            check=False,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired:
        result.timed_out = True
        return result

    result.return_code = completed.returncode
    return result


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
    copied_paths.extend(_copy_skills_tree(workspace_dir))

    if is_planner_agent(agent_name):
        copied_paths.extend(
            _write_template_files(workspace_dir, load_role_template_files(agent_name))
        )
        copied_paths.extend(
            _write_template_files(workspace_dir, load_codex_template_files())
        )
    elif is_coder_agent(agent_name):
        copied_paths.extend(
            _write_template_files(
                workspace_dir,
                {
                    rel_path: content
                    for rel_path, content in load_codex_template_files().items()
                    if rel_path.startswith("scripts/submit_for_review.")
                },
            )
        )
    elif is_reviewer_agent(agent_name):
        copied_paths.extend(
            _write_template_files(
                workspace_dir,
                {
                    rel_path: content
                    for rel_path, content in load_codex_template_files().items()
                    if rel_path.startswith("scripts/submit_review.")
                },
            )
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

    if agent_name in _ENGINEER_DRAFTING_TARGETS and _engineering_drafting_mode_active():
        copied_paths.extend(
            _write_missing_template_files(
                workspace_dir, load_template_repo_files("engineer/drafting")
            )
        )
        copied_paths.extend(_ensure_engineer_drafting_contract(workspace_dir))
    if (
        agent_name in _ENGINEER_BENCHMARK_CONTEXT_TARGETS
        and _benchmark_drafting_mode_active()
    ):
        copied_paths.extend(
            _write_missing_template_files(
                workspace_dir, load_template_repo_files("benchmark_generator/drafting")
            )
        )
    if agent_name in _BENCHMARK_DRAFTING_TARGETS and _benchmark_drafting_mode_active():
        copied_paths.extend(
            _write_missing_template_files(
                workspace_dir, load_template_repo_files("benchmark_generator/drafting")
            )
        )
        copied_paths.extend(_ensure_benchmark_drafting_contract(workspace_dir))

    if (
        agent_name
        not in {
            AgentName.BENCHMARK_PLANNER,
            AgentName.BENCHMARK_PLAN_REVIEWER,
        }
        and (workspace_dir / "benchmark_definition.yaml").exists()
    ):
        benchmark_script_path = workspace_dir / "benchmark_script.py"
        if not benchmark_script_path.exists():
            benchmark_script_text = load_template_repo_files("benchmark_generator")[
                "benchmark_script.py"
            ]
            copied_paths.extend(
                _write_template_files(
                    workspace_dir,
                    {"benchmark_script.py": benchmark_script_text},
                )
            )

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
    elif is_coder_agent(agent_name):
        helper_script_paths.append("scripts/submit_for_review.sh")
    elif is_reviewer_agent(agent_name):
        helper_script_paths.append("scripts/submit_review.sh")

    return MaterializedWorkspace(
        workspace_dir=workspace_dir,
        prompt_path=prompt_path,
        prompt_text=prompt_text,
        agent_name=agent_name,
        task_id=item.id,
        copied_paths=copied_paths,
        helper_script_paths=helper_script_paths,
    )


def build_codex_env(
    *,
    task_id: str,
    workspace_dir: Path,
    codex_home_root: Path,
    session_id: str | None = None,
    agent_name: AgentName | None = None,
) -> dict[str, str]:
    """Prepare a generic Codex subprocess environment for a local workspace run."""

    env = dict(os.environ)
    # Codex runs are headless by contract. Strip ambient display state so the
    # submission helpers and validation paths cannot depend on host X11/Wayland.
    env.pop("DISPLAY", None)
    env.pop("XAUTHORITY", None)
    env.pop("WAYLAND_DISPLAY", None)
    env.setdefault("PROBLEMOLOGIST_PHYSICS_GL_BACKEND", "egl")
    env.setdefault("PROBLEMOLOGIST_RENDER_GL_BACKEND", "osmesa")
    headless_config = load_headless_opengl_config(env)
    headless_config.apply(env)
    env["HOME"] = str(codex_home_root)
    env["CODEX_HOME"] = str(codex_home_root / ".codex")
    env["XDG_CACHE_HOME"] = str(codex_home_root / ".cache")
    env["XDG_CONFIG_HOME"] = str(codex_home_root / ".config")
    env.setdefault("TMPDIR", str(codex_home_root / ".tmp"))
    env.setdefault("TEMP", str(codex_home_root / ".tmp"))
    env.setdefault("TMP", str(codex_home_root / ".tmp"))
    workspace_path = workspace_dir.expanduser().resolve()

    venv_bin = ROOT / ".venv" / "bin"
    if venv_bin.exists():
        current_path = env.get("PATH", "")
        env["PATH"] = (
            f"{venv_bin}{os.pathsep}{current_path}" if current_path else str(venv_bin)
        )
        env.setdefault("VIRTUAL_ENV", str(ROOT / ".venv"))
        env["PYTHON_BIN"] = str(venv_bin / "python")
    else:
        env.setdefault("PYTHON_BIN", sys.executable)

    env.setdefault("CONTROLLER_URL", "http://localhost:18000")
    env.setdefault("WORKER_LIGHT_URL", "http://localhost:18001")
    env.setdefault("SESSION_ID", session_id or f"local-codex-{task_id}-{os.getpid()}")
    env.setdefault("IS_HEAVY_WORKER", "1")
    env.setdefault("PROBLEMOLOGIST_SCRIPT_IMPORT_MODE", "0")
    env.setdefault("COTS_DB_PATH", str(ROOT / "parts.db"))
    env.setdefault("PROBLEMOLOGIST_REPO_ROOT", str(ROOT))
    current_revision = repo_revision(ROOT)
    if current_revision:
        env.setdefault("REPO_REVISION", current_revision)
    existing_pythonpath = env.get("PYTHONPATH", "")
    pythonpath_entries = [str(workspace_path), str(ROOT)]
    if existing_pythonpath:
        pythonpath_entries.append(existing_pythonpath)
    env["PYTHONPATH"] = os.pathsep.join(entry for entry in pythonpath_entries if entry)
    if agent_name is not None:
        env.setdefault("AGENT_NAME", agent_name.value)
    return env


def _codex_execution_flags(*, yolo: bool) -> list[str]:
    if yolo:
        return ["--dangerously-bypass-approvals-and-sandbox"]
    return ["--full-auto"]


def launch_codex_exec(
    workspace_dir: Path,
    prompt_text: str,
    *,
    task_id: str,
    agent_name: AgentName | None = None,
    session_id: str | None = None,
    runtime_root: Path | None = None,
    yolo: bool = True,
    timeout_seconds: float | None = None,
    reasoning_effort: Literal["low", "medium", "high", "xhigh"]
    | None
    | object = _REASONING_EFFORT_UNSET,
) -> int:
    """Launch `codex exec` in a workspace and stream output to the terminal."""
    result = _run_codex_exec_command(
        workspace_dir=workspace_dir,
        prompt_text=prompt_text,
        task_id=task_id,
        agent_name=agent_name,
        session_id=session_id,
        runtime_root=runtime_root,
        yolo=yolo,
        timeout_seconds=timeout_seconds,
        reasoning_effort=reasoning_effort,
    )
    if result.timed_out:
        return 124
    return result.return_code if result.return_code is not None else 1


def resume_codex_exec(
    workspace_dir: Path,
    prompt_text: str,
    *,
    task_id: str,
    codex_session_id: str,
    agent_name: AgentName | None = None,
    session_id: str | None = None,
    runtime_root: Path | None = None,
    yolo: bool = True,
    timeout_seconds: float | None = None,
    output_last_message_path: Path | None = None,
    reasoning_effort: Literal["low", "medium", "high", "xhigh"]
    | None
    | object = _REASONING_EFFORT_UNSET,
) -> CodexExecRunResult:
    """Resume an existing `codex exec` session with a follow-up prompt."""

    return _run_codex_exec_command(
        workspace_dir=workspace_dir,
        prompt_text=prompt_text,
        task_id=task_id,
        agent_name=agent_name,
        session_id=session_id,
        runtime_root=runtime_root,
        yolo=yolo,
        timeout_seconds=timeout_seconds,
        resume_session_id=codex_session_id,
        output_last_message_path=output_last_message_path,
        reasoning_effort=reasoning_effort,
    )


def open_codex_ui(
    workspace_dir: Path,
    prompt_text: str,
    *,
    task_id: str,
    agent_name: AgentName | None = None,
    session_id: str | None = None,
    runtime_root: Path | None = None,
    yolo: bool = True,
) -> int:
    """Open the interactive Codex UI with a workspace prompt."""
    if not (workspace_dir / ".git").exists():
        subprocess.run(["git", "init", "-q", str(workspace_dir)], check=False)
    codex_home_root = resolve_codex_home_root(
        task_id=task_id,
        session_id=session_id,
        runtime_root=runtime_root,
    )
    prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        agent_name=agent_name,
    )
    cmd = [
        "codex",
        *_codex_execution_flags(yolo=yolo),
        "-c",
        "shell_environment_policy.inherit=all",
        "--cd",
        str(workspace_dir),
        "--no-alt-screen",
        prompt_text,
    ]
    print("launching: " + " ".join(cmd))
    completed = subprocess.run(
        cmd,
        env=build_codex_env(
            task_id=task_id,
            workspace_dir=workspace_dir,
            codex_home_root=codex_home_root,
            session_id=session_id,
            agent_name=agent_name,
        ),
        check=False,
    )
    return completed.returncode


def _load_review_frontmatter_local(
    *,
    workspace_dir: Path,
    review_filename_prefix: str,
    session_id: str,
) -> tuple[ReviewFrontmatter | None, str | None]:
    review_data, _, review_error = _load_review_artifacts_local(
        workspace_dir=workspace_dir,
        review_filename_prefix=review_filename_prefix,
        session_id=session_id,
    )
    return review_data, review_error


def _load_review_artifacts_local(
    *,
    workspace_dir: Path,
    review_filename_prefix: str,
    session_id: str,
) -> tuple[ReviewFrontmatter | None, ReviewComments | None, str | None]:
    reviews_dir = workspace_dir / "reviews"
    if not reviews_dir.exists():
        return None, None, "reviews/ directory not found"

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
            return None, None, f"failed to read {review_path}: {exc}"

        review_data, review_error = parse_review_decision_yaml(
            content,
            review_path=review_path.as_posix(),
        )
        if review_error is not None:
            return None, None, review_error
        if review_data is None:
            return None, None, f"missing review decision payload in {review_path}"

        comments_path = (
            reviews_dir
            / f"{latest_yaml[1].stem.replace('-decision-round-', '-comments-round-')}.yaml"
        )
        if not comments_path.exists():
            return (
                review_data,
                None,
                f"missing review comments payload in {comments_path}",
            )
        try:
            comments_content = comments_path.read_text(encoding="utf-8")
        except Exception as exc:
            return None, None, f"failed to read {comments_path}: {exc}"
        comments_data, comments_error = _parse_review_comments_yaml(
            comments_content,
            comments_path=comments_path.as_posix(),
        )
        if comments_error is not None:
            return None, None, comments_error
        if comments_data is None:
            return None, None, f"missing review comments payload in {comments_path}"
        return review_data, comments_data, None

    if latest_md is not None:
        review_path = latest_md[1]
        try:
            content = review_path.read_text(encoding="utf-8")
        except Exception as exc:
            return None, None, f"failed to read {review_path}: {exc}"

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
            return None, None, f"invalid review frontmatter in {review_path}: {details}"
        if not isinstance(review_data, ReviewFrontmatter):
            return (
                None,
                None,
                f"unexpected review frontmatter payload for {review_path}",
            )
        return review_data, None, None

    return (
        None,
        None,
        "no persisted review artifact matching "
        f"reviews/{review_filename_prefix}-decision-round-<n>.yaml "
        f"or reviews/{review_filename_prefix}-<n>.md",
    )


def _parse_review_comments_yaml(
    content: str, *, review_path: str
) -> tuple[ReviewComments | None, str | None]:
    try:
        data = yaml.safe_load(content)
    except yaml.YAMLError as exc:
        return None, f"invalid YAML in {review_path}: {exc}"

    if not isinstance(data, dict):
        return None, f"unexpected YAML structure in {review_path}: expected mapping"

    try:
        return ReviewComments.model_validate(data), None
    except Exception as exc:
        return None, f"invalid review comments payload in {review_path}: {exc}"


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
    required_helper_files = [
        workspace_dir / "scripts" / "submit_for_review.sh",
        workspace_dir / "scripts" / "submit_for_review.py",
    ]
    missing_helpers = [
        str(path.relative_to(workspace_dir))
        for path in required_helper_files
        if not path.exists()
    ]
    if missing_helpers:
        return WorkspaceVerificationResult(
            success=False,
            verification_name="coder_workspace_contract",
            errors=[
                "missing Codex submission helper file(s): " + ", ".join(missing_helpers)
            ],
        )
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

    review_data, comments_data, review_error = _load_review_artifacts_local(
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
            "review_decision": (
                review_data.model_dump(mode="json") if review_data is not None else None
            ),
            "review_comments": (
                comments_data.model_dump(mode="json")
                if comments_data is not None
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
