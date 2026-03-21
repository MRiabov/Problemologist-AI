"""Materialize a seeded eval row into a local temp workspace.

This is an inspection helper, not the eval runner. It copies the seeded
artifacts for a single dataset row into a persistent temp directory and writes
the row prompt to `prompt.md` so you can inspect the exact workspace contents
an agent would start from.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from evals.logic.models import EvalDatasetItem  # noqa: E402
from shared.agent_templates import load_common_template_files  # noqa: E402
from shared.enums import AgentName  # noqa: E402

DATASET_ROOTS = (
    ROOT / "dataset" / "evals" / "datasets",
    ROOT / "dataset" / "data" / "seed" / "role_based",
)
VENV_BIN = ROOT / ".venv" / "bin"


def _resolve_seed_artifact_dir(item: EvalDatasetItem) -> Path | None:
    if item.seed_artifact_dir is None:
        return None

    artifact_dir = Path(item.seed_artifact_dir)
    if artifact_dir.is_absolute():
        return artifact_dir

    repo_relative = ROOT / artifact_dir
    if repo_relative.exists():
        return repo_relative

    if item.seed_dataset is not None:
        dataset_relative = (ROOT / item.seed_dataset).parent / artifact_dir
        if dataset_relative.exists():
            return dataset_relative

    return repo_relative


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a local temp workspace from a seeded eval row and write "
            "prompt.md for inspection."
        )
    )
    parser.add_argument(
        "--agent",
        required=True,
        help="Agent dataset to materialize, for example engineer_coder.",
    )
    parser.add_argument(
        "--task-id",
        default=None,
        help="Specific dataset row ID to materialize. Required when the agent has multiple rows.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=(
            "Optional destination directory. If omitted, a persistent tempdir "
            "is created with tempfile.mkdtemp()."
        ),
    )
    parser.add_argument(
        "--launch-codex",
        action="store_true",
        help=(
            "After materializing the workspace, launch `codex exec` in that "
            "directory using prompt.md as the prompt."
        ),
    )
    parser.add_argument(
        "--open-codex",
        "--open",
        dest="open_codex",
        action="store_true",
        help=(
            "After materializing the workspace, open the interactive Codex UI "
            "in that directory using prompt.md as the initial prompt."
        ),
    )
    parser.add_argument(
        "--env-up",
        action="store_true",
        help=(
            "Run scripts/env_up.sh before launching Codex so the controller and "
            "worker services are available on the local ports."
        ),
    )
    yolo_group = parser.add_mutually_exclusive_group()
    yolo_group.add_argument(
        "--yolo",
        dest="yolo",
        action="store_true",
        help=("Launch Codex with approvals and sandbox bypassed. This is the default."),
    )
    yolo_group.add_argument(
        "--no-yolo",
        dest="yolo",
        action="store_false",
        help="Launch Codex with normal approval/sandbox behavior.",
    )
    parser.set_defaults(yolo=True)
    return parser.parse_args()


def _load_dataset(agent: AgentName) -> tuple[Path, list[EvalDatasetItem]]:
    json_path = next(
        (
            root / f"{agent.value}.json"
            for root in DATASET_ROOTS
            if (root / f"{agent.value}.json").exists()
        ),
        None,
    )
    if json_path is None:
        searched = ", ".join(str(path) for path in DATASET_ROOTS)
        raise FileNotFoundError(
            f"Dataset for agent '{agent.value}' not found. Searched: {searched}"
        )

    with json_path.open(encoding="utf-8") as handle:
        raw_rows = json.load(handle)

    seed_dataset = json_path.relative_to(ROOT)
    rows = [
        EvalDatasetItem.model_validate({**row, "seed_dataset": seed_dataset})
        for row in raw_rows
    ]
    return json_path, rows


def _select_row(
    rows: list[EvalDatasetItem], *, task_id: str | None, agent: AgentName
) -> EvalDatasetItem:
    if task_id:
        for row in rows:
            if row.id == task_id:
                return row
        available = ", ".join(row.id for row in rows)
        raise SystemExit(
            f"Task id '{task_id}' was not found for agent '{agent.value}'. "
            f"Available rows: {available}"
        )

    if len(rows) == 1:
        return rows[0]

    available = ", ".join(row.id for row in rows)
    raise SystemExit(
        f"Agent '{agent.value}' has multiple rows. Pass --task-id. Available rows: {available}"
    )


def _ensure_destination(
    output_dir: str | None, *, agent: AgentName, task_id: str
) -> Path:
    if output_dir:
        destination = Path(output_dir).expanduser().resolve()
        destination.mkdir(parents=True, exist_ok=True)
        return destination

    prefix = f"problemologist-{agent.value}-{task_id}-"
    return Path(tempfile.mkdtemp(prefix=prefix))


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


def _write_inline_files(dst_root: Path, seed_files: dict[str, str] | None) -> list[str]:
    if not seed_files:
        return []

    copied: list[str] = []
    for rel_path, content in sorted(seed_files.items()):
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(content, encoding="utf-8")
        copied.append(rel_path)
    return copied


def _write_template_files(dst_root: Path) -> list[str]:
    copied: list[str] = []
    for rel_path, content in load_common_template_files().items():
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(content, encoding="utf-8")
        copied.append(rel_path)
    return copied


def _write_prompt(dst_root: Path, task: str) -> None:
    prompt_path = dst_root / "prompt.md"
    prompt_path.write_text(task.rstrip() + "\n", encoding="utf-8")


def _build_codex_env(*, agent: AgentName, task_id: str) -> dict[str, str]:
    env = dict(os.environ)
    py_path = env.get("PYTHONPATH")
    env["PYTHONPATH"] = f"{ROOT}{os.pathsep}{py_path}" if py_path else str(ROOT)
    if VENV_BIN.exists():
        current_path = env.get("PATH", "")
        env["PATH"] = (
            f"{VENV_BIN}{os.pathsep}{current_path}" if current_path else str(VENV_BIN)
        )
        env.setdefault("VIRTUAL_ENV", str(ROOT / ".venv"))
    env.setdefault("CONTROLLER_URL", "http://localhost:18000")
    env.setdefault("WORKER_LIGHT_URL", "http://localhost:18001")
    env.setdefault("AGENT_NAME", agent.value)
    env.setdefault("SESSION_ID", f"local-codex-{agent.value}-{task_id}-{os.getpid()}")
    return env


def _launch_codex_exec(
    workspace_dir: Path,
    prompt: str,
    *,
    agent: AgentName,
    task_id: str,
    yolo: bool,
) -> int:
    cmd = [
        "codex",
        "exec",
        "-c",
        "shell_environment_policy.inherit=all",
        "--cd",
        str(workspace_dir),
        "--skip-git-repo-check",
    ]
    if yolo:
        cmd.append("--yolo")
    else:
        cmd.append("--full-auto")
    cmd.append("-")
    print("launching: " + " ".join(cmd))
    completed = subprocess.run(
        cmd,
        input=prompt,
        text=True,
        env=_build_codex_env(agent=agent, task_id=task_id),
        check=False,
    )
    return completed.returncode


def _open_codex_ui(
    workspace_dir: Path,
    prompt: str,
    *,
    agent: AgentName,
    task_id: str,
    yolo: bool,
) -> int:
    if not (workspace_dir / ".git").exists():
        subprocess.run(
            ["git", "init", "-q", str(workspace_dir)],
            check=False,
            env=dict(os.environ),
        )
    cmd = [
        "codex",
        "-c",
        "shell_environment_policy.inherit=all",
        "--cd",
        str(workspace_dir),
        "--no-alt-screen",
    ]
    if yolo:
        cmd.insert(1, "--yolo")
    else:
        cmd.insert(1, "--full-auto")
    cmd.append(prompt)
    print("launching: " + " ".join(cmd))
    completed = subprocess.run(
        cmd,
        env=_build_codex_env(agent=agent, task_id=task_id),
        check=False,
    )
    return completed.returncode


def _env_up() -> None:
    env_up_path = ROOT / "scripts" / "env_up.sh"
    if not env_up_path.exists():
        raise FileNotFoundError(f"Missing env bootstrap script: {env_up_path}")

    print(f"bootstrapping eval environment with: {env_up_path}")
    completed = subprocess.run([str(env_up_path)], check=False, env=dict(os.environ))
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def main() -> None:
    args = _parse_args()
    try:
        agent = AgentName(args.agent)
    except ValueError as exc:
        available = ", ".join(sorted(agent.value for agent in list(AgentName)))
        raise SystemExit(
            f"Unknown agent '{args.agent}'. Available: {available}"
        ) from exc

    _, rows = _load_dataset(agent)
    row = _select_row(rows, task_id=args.task_id, agent=agent)

    workspace_dir = _ensure_destination(args.output_dir, agent=agent, task_id=row.id)

    copied_paths: list[str] = []
    copied_paths.extend(_write_template_files(workspace_dir))
    artifact_dir = _resolve_seed_artifact_dir(row)
    if artifact_dir is not None:
        if not artifact_dir.exists():
            raise FileNotFoundError(
                f"Seed artifact directory not found: {artifact_dir}"
            )
        copied_paths.extend(_copy_tree(artifact_dir, workspace_dir))

    copied_paths.extend(_write_inline_files(workspace_dir, row.seed_files))
    _write_prompt(workspace_dir, row.task)
    copied_paths.append("prompt.md")

    print(f"workspace: {workspace_dir}")
    print(f"agent: {agent.value}")
    print(f"task_id: {row.id}")
    print("files:")
    for rel_path in copied_paths:
        print(f"  - {rel_path}")

    if args.launch_codex and args.open_codex:
        raise SystemExit("Choose only one of --launch-codex or --open-codex.")

    if args.env_up:
        _env_up()

    if args.launch_codex:
        raise SystemExit(
            _launch_codex_exec(
                workspace_dir,
                row.task,
                agent=agent,
                task_id=row.id,
                yolo=args.yolo,
            )
        )

    if args.open_codex:
        raise SystemExit(
            _open_codex_ui(
                workspace_dir,
                row.task,
                agent=agent,
                task_id=row.id,
                yolo=args.yolo,
            )
        )


if __name__ == "__main__":
    main()
