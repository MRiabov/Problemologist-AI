"""Materialize a seeded eval row into a local temp workspace.

This is an inspection helper, not the eval runner. It copies the seeded
artifacts for a single dataset row into a persistent temp directory and writes
the Codex prompt to `prompt.md` so you can inspect the exact workspace contents
an agent would start from.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from evals.logic.codex_workspace import (  # noqa: E402
    launch_codex_exec,
    open_codex_ui,
)
from evals.logic.codex_workspace import (
    materialize_seed_workspace as materialize_codex_workspace,
)
from evals.logic.models import EvalDatasetItem  # noqa: E402
from shared.enums import AgentName  # noqa: E402

DATASET_ROOTS = (
    ROOT / "dataset" / "evals" / "datasets",
    ROOT / "dataset" / "data" / "seed" / "role_based",
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a local temp workspace from a seeded eval row and write "
            "prompt.md for inspection or Codex launching."
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
    materialized = materialize_codex_workspace(
        item=row,
        agent_name=agent,
        workspace_dir=workspace_dir,
    )

    print(f"workspace: {materialized.workspace_dir}")
    print(f"prompt: {materialized.prompt_path}")
    print(f"agent: {agent.value}")
    print(f"task_id: {row.id}")
    print("files:")
    for rel_path in materialized.copied_paths:
        print(f"  - {rel_path}")

    if args.launch_codex and args.open_codex:
        raise SystemExit("Choose only one of --launch-codex or --open-codex.")

    if args.env_up:
        _env_up()

    if args.launch_codex:
        raise SystemExit(
            launch_codex_exec(
                materialized.workspace_dir,
                materialized.prompt_text,
                agent_name=agent,
                task_id=row.id,
                yolo=args.yolo,
            )
        )

    if args.open_codex:
        raise SystemExit(
            open_codex_ui(
                materialized.workspace_dir,
                materialized.prompt_text,
                agent_name=agent,
                task_id=row.id,
                yolo=args.yolo,
            )
        )


if __name__ == "__main__":
    main()
