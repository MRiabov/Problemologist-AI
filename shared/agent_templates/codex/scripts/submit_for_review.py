from __future__ import annotations

import json
import os
from importlib import import_module
from pathlib import Path
from typing import Any

from build123d import Compound

from shared.enums import AgentName
from shared.workers.persistence import record_validation_result
from utils.submission import simulate, validate
from worker_heavy.utils.handover import submit_for_review as _handover_submit_for_review


def _coder_agent(workspace: Path) -> AgentName | None:
    benchmark_handoff = workspace / "benchmark_assembly_definition.yaml"
    engineering_handoff = workspace / "assembly_definition.yaml"

    has_benchmark_handoff = benchmark_handoff.exists()
    has_engineering_handoff = engineering_handoff.exists()

    if has_engineering_handoff:
        return AgentName.ENGINEER_CODER
    if has_benchmark_handoff and not has_engineering_handoff:
        return AgentName.BENCHMARK_CODER
    return None


def _load_solution() -> Compound:
    module = import_module("script")
    solution = getattr(module, "result", None)
    if solution is None and hasattr(module, "build"):
        solution = module.build()
    if solution is None:
        raise ValueError("script.py must define module-level result or build()")
    if not isinstance(solution, Compound):
        raise TypeError(
            f"script.py must export a build123d.Compound; got {type(solution).__name__}"
        )
    return solution


def _print_json(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True))


def main() -> int:
    workspace = Path.cwd()
    session_id = os.getenv("SESSION_ID")
    episode_id = os.getenv("EPISODE_ID") or None
    agent_name = _coder_agent(workspace)
    if agent_name is None:
        _print_json(
            {
                "ok": False,
                "status": "rejected",
                "stage": "load",
                "message": (
                    "Unable to infer coder agent from workspace: expected "
                    "benchmark_assembly_definition.yaml or assembly_definition.yaml"
                ),
            }
        )
        return 1

    try:
        solution = _load_solution()
    except Exception as exc:
        _print_json(
            {
                "ok": False,
                "status": "rejected",
                "stage": "load",
                "message": str(exc),
            }
        )
        return 1

    validation_ok, validation_message = validate(
        solution,
        output_dir=workspace,
        session_id=session_id,
    )
    record_validation_result(
        workspace,
        validation_ok,
        validation_message,
        script_path="script.py",
        session_id=session_id,
    )
    if not validation_ok:
        _print_json(
            {
                "ok": False,
                "status": "rejected",
                "stage": "validation",
                "message": validation_message,
            }
        )
        return 1

    simulation_result = simulate(
        solution,
        output_dir=workspace,
        session_id=session_id,
    )
    if not simulation_result.success:
        _print_json(
            {
                "ok": False,
                "status": "rejected",
                "stage": "simulation",
                "message": simulation_result.summary,
            }
        )
        return 1

    submitted = _handover_submit_for_review(
        solution,
        cwd=workspace,
        session_id=session_id,
        reviewer_stage=(
            AgentName.ENGINEER_EXECUTION_REVIEWER
            if agent_name == AgentName.ENGINEER_CODER
            else AgentName.BENCHMARK_REVIEWER
        ),
        episode_id=episode_id,
        script_path="script.py",
    )
    if not submitted:
        _print_json(
            {
                "ok": False,
                "status": "rejected",
                "stage": "handover",
                "message": "submit_for_review returned false",
            }
        )
        return 1

    manifest_path = (
        workspace / ".manifests" / "engineering_execution_review_manifest.json"
    )
    _print_json(
        {
            "ok": True,
            "status": "submitted",
            "stage": (
                "engineering_execution_reviewer"
                if agent_name == AgentName.ENGINEER_CODER
                else "benchmark_reviewer"
            ),
            "manifest_path": str(manifest_path.relative_to(workspace))
            if manifest_path.exists()
            else None,
            "validation_success": validation_ok,
            "simulation_success": simulation_result.success,
        }
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
