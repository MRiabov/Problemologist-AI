from __future__ import annotations

import argparse
import asyncio
import os
import subprocess
import sys
import tempfile
import uuid
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from shared.agents.config import DraftingMode  # noqa: E402
from shared.enums import AgentName, ReviewDecision  # noqa: E402
from shared.logging import get_logger  # noqa: E402

logger = get_logger(__name__)
CLI_PROVIDERS = ("codex", "qwen")

DEFAULT_SEED_DIR = (
    ROOT / "dataset" / "data" / "seed" / "artifacts" / "e2e" / "e2e-001-drawing-full"
)
DEFAULT_TASK_ID = "e2e-001-drawing-full"


@dataclass(frozen=True, slots=True)
class StageSpec:
    agent_name: AgentName
    expected_decision: ReviewDecision | None = None
    requires_simulation_result: bool = False


@dataclass(slots=True)
class StageRunResult:
    agent_name: AgentName
    workspace_dir: Path
    launch_return_code: int
    verification_name: str
    verification_success: bool
    simulation_success: bool | None
    review_decision: str | None


STAGES: tuple[StageSpec, ...] = (
    StageSpec(AgentName.BENCHMARK_PLANNER),
    StageSpec(AgentName.BENCHMARK_PLAN_REVIEWER, ReviewDecision.APPROVED),
    StageSpec(AgentName.BENCHMARK_CODER, requires_simulation_result=True),
    StageSpec(AgentName.BENCHMARK_REVIEWER, ReviewDecision.APPROVED),
    StageSpec(AgentName.ENGINEER_PLANNER),
    StageSpec(AgentName.ENGINEER_PLAN_REVIEWER, ReviewDecision.APPROVED),
    StageSpec(AgentName.ENGINEER_CODER, requires_simulation_result=True),
    StageSpec(AgentName.ENGINEER_EXECUTION_REVIEWER, ReviewDecision.APPROVED),
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the benchmark -> engineer smoke path from a human-written "
            "prompt seed using the local CLI provider helpers."
        )
    )
    parser.add_argument(
        "--seed-dir",
        type=Path,
        default=DEFAULT_SEED_DIR,
        help=(
            "Seed artifact directory containing human-written-description.md "
            f"(default: {DEFAULT_SEED_DIR.relative_to(ROOT)})"
        ),
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=None,
        help=(
            "Optional root directory for stage workspaces and CLI runtime state. "
            "If omitted, a persistent temp directory is created."
        ),
    )
    parser.add_argument(
        "--provider",
        type=str,
        default=os.getenv("PROBLEMOLOGIST_CLI_PROVIDER", "qwen"),
        choices=CLI_PROVIDERS,
        help="Local CLI provider to launch for each stage.",
    )
    parser.add_argument(
        "--yolo",
        action="store_true",
        help="Launch the CLI provider in bypass-approval mode.",
    )
    parser.add_argument(
        "--env-up",
        action="store_true",
        help="Run scripts/env_up.sh --profile eval before the e2e chain starts.",
    )
    parser.add_argument(
        "--open-cli-ui",
        action="store_true",
        help=(
            "Open the selected CLI provider UI on each materialized stage "
            "workspace before launching that stage."
        ),
    )
    parser.add_argument(
        "--new-terminal",
        action="store_true",
        help=(
            "When used with --open-cli-ui, open the UI in a separate terminal "
            "window instead of the current shell."
        ),
    )
    return parser.parse_args()


def _run_env_up() -> None:
    env_up_script = ROOT / "scripts" / "env_up.sh"
    subprocess.run(
        [str(env_up_script), "--profile", "eval"],
        cwd=ROOT,
        check=True,
    )


def _read_seed_prompt(seed_dir: Path) -> str:
    prompt_path = seed_dir / "human-written-description.md"
    if not prompt_path.exists():
        raise FileNotFoundError(f"Seed prompt missing: {prompt_path}")
    return prompt_path.read_text(encoding="utf-8").strip()


def _build_base_item(seed_dir: Path, prompt_text: str) -> EvalDatasetItem:
    from evals.logic.models import EvalDatasetItem

    try:
        seed_dataset = seed_dir.relative_to(ROOT)
    except ValueError:
        seed_dataset = None

    return EvalDatasetItem.model_validate(
        {
            "id": DEFAULT_TASK_ID,
            "task": prompt_text,
            "complexity_level": 0,
            "seed_dataset": seed_dataset,
            "seed_artifact_dir": seed_dir,
            "technical_drawing_mode": DraftingMode.FULL,
        }
    )


def _simulation_success(workspace_dir: Path) -> bool | None:
    simulation_path = workspace_dir / "simulation_result.json"
    if not simulation_path.exists():
        return None
    try:
        payload = simulation_path.read_text(encoding="utf-8")
        from shared.models.simulation import SimulationResult

        return bool(SimulationResult.model_validate_json(payload).success)
    except Exception:
        return False


def _format_verification_errors(errors: list[str]) -> str:
    if not errors:
        return "unknown verification failure"
    return "; ".join(errors)


async def _run_stage(
    *,
    stage: StageSpec,
    item: EvalDatasetItem,
    workspace_root: Path,
    provider_name: str,
    yolo: bool,
    source_seed_dir: Path,
    open_cli_ui_requested: bool,
    open_cli_ui_new_terminal: bool,
) -> StageRunResult:
    from evals.logic.codex_workspace import (
        LocalWorkspaceClient,
        launch_cli_exec,
        materialize_seed_workspace,
        open_cli_ui,
        verify_workspace_for_agent,
    )
    from evals.logic.specs import AGENT_SPECS
    from evals.logic.workspace import preflight_seeded_entry_contract

    stage_workspace_dir = (
        workspace_root
        / "workspaces"
        / f"{stage.agent_name.value}-{uuid.uuid4().hex[:8]}"
    )
    stage_workspace_dir.parent.mkdir(parents=True, exist_ok=True)

    stage_item = item.model_copy(update={"seed_artifact_dir": source_seed_dir})
    materialized = materialize_seed_workspace(
        item=stage_item,
        agent_name=stage.agent_name,
        workspace_dir=stage_workspace_dir,
    )

    session_id = f"e2e-seed-{stage.agent_name.value}-{item.id}-{uuid.uuid4().hex[:8]}"
    worker_client = LocalWorkspaceClient(
        root=materialized.workspace_dir,
        session_id=session_id,
    )
    spec = AGENT_SPECS[stage.agent_name]
    await preflight_seeded_entry_contract(
        item=stage_item,
        session_id=session_id,
        agent_name=stage.agent_name,
        spec=spec,
        root=ROOT,
        worker_light_url=os.getenv("WORKER_LIGHT_URL", "http://localhost:18001"),
        logger=logger,
        workspace_client=worker_client,
    )

    if open_cli_ui_requested:
        ui_return_code = await asyncio.to_thread(
            open_cli_ui,
            materialized.workspace_dir,
            materialized.prompt_text,
            task_id=item.id,
            agent_name=stage.agent_name,
            session_id=session_id,
            runtime_root=workspace_root,
            yolo=yolo,
            provider_name=provider_name,
            new_terminal=open_cli_ui_new_terminal,
        )
        if ui_return_code != 0:
            raise SystemExit(
                f"open_cli_ui exited with {ui_return_code} for {stage.agent_name.value}"
            )

    launch_return_code = await asyncio.to_thread(
        launch_cli_exec,
        materialized.workspace_dir,
        materialized.prompt_text,
        task_id=item.id,
        agent_name=stage.agent_name,
        session_id=session_id,
        runtime_root=workspace_root,
        yolo=yolo,
        provider_name=provider_name,
    )

    verification = await verify_workspace_for_agent(
        workspace_dir=materialized.workspace_dir,
        agent_name=stage.agent_name,
        session_id=session_id,
        expected_decision=stage.expected_decision,
    )
    sim_success = _simulation_success(materialized.workspace_dir)

    review_decision = None
    details = verification.details if isinstance(verification.details, dict) else {}
    review_data = details.get("review_decision") if isinstance(details, dict) else None
    if isinstance(review_data, dict):
        review_decision = str(review_data.get("decision") or "").strip() or None

    return StageRunResult(
        agent_name=stage.agent_name,
        workspace_dir=materialized.workspace_dir,
        launch_return_code=launch_return_code,
        verification_name=verification.verification_name,
        verification_success=verification.success,
        simulation_success=sim_success,
        review_decision=review_decision,
    )


def _print_stage_result(result: StageRunResult) -> None:
    print(
        f"[{result.agent_name.value}] workspace={result.workspace_dir} "
        f"launch_rc={result.launch_return_code} "
        f"verification={result.verification_name}:{result.verification_success} "
        f"simulation_success={result.simulation_success} "
        f"review_decision={result.review_decision}"
    )


async def _run_chain(args: argparse.Namespace) -> int:
    if args.env_up:
        _run_env_up()

    seed_dir = args.seed_dir.expanduser().resolve()
    prompt_text = _read_seed_prompt(seed_dir)
    base_item = _build_base_item(seed_dir, prompt_text)

    if args.workspace_root is None:
        workspace_root = Path(tempfile.mkdtemp(prefix="problemologist-e2e-seed-"))
    else:
        workspace_root = args.workspace_root.expanduser().resolve()
        workspace_root.mkdir(parents=True, exist_ok=True)

    print(f"workspace_root={workspace_root}")
    print(f"seed_dir={seed_dir}")

    source_seed_dir = seed_dir
    last_result: StageRunResult | None = None
    for stage in STAGES:
        result = await _run_stage(
            stage=stage,
            item=base_item,
            workspace_root=workspace_root,
            provider_name=args.provider,
            yolo=args.yolo,
            source_seed_dir=source_seed_dir,
            open_cli_ui_requested=args.open_cli_ui,
            open_cli_ui_new_terminal=args.new_terminal,
        )
        _print_stage_result(result)

        failure_reasons: list[str] = []
        if result.launch_return_code != 0:
            failure_reasons.append(f"launch exited with {result.launch_return_code}")
        if not result.verification_success:
            failure_reasons.append(f"verification failed: {result.verification_name}")
        if stage.requires_simulation_result:
            if result.simulation_success is False:
                failure_reasons.append("simulation_result.json reported failure")
            if result.simulation_success is None:
                failure_reasons.append("simulation_result.json missing")
        if stage.expected_decision is not None and result.review_decision is None:
            failure_reasons.append(
                f"review decision missing: expected {stage.expected_decision.value}"
            )
        if (
            stage.expected_decision is not None
            and result.review_decision is not None
            and result.review_decision != stage.expected_decision.value
        ):
            failure_reasons.append(
                f"review decision mismatch: expected {stage.expected_decision.value}, "
                f"got {result.review_decision}"
            )

        if failure_reasons:
            print(
                f"[{stage.agent_name.value}] failed: "
                + _format_verification_errors(failure_reasons)
            )
            return 1

        source_seed_dir = result.workspace_dir
        last_result = result

    if last_result is not None:
        print(f"final_workspace={last_result.workspace_dir}")
    return 0


def main() -> None:
    args = _parse_args()
    if args.new_terminal and not args.open_cli_ui:
        raise SystemExit("--new-terminal requires --open-cli-ui.")
    raise SystemExit(asyncio.run(_run_chain(args)))


if __name__ == "__main__":
    main()
