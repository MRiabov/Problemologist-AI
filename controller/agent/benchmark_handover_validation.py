from __future__ import annotations

from typing import Any

from controller.clients.worker import WorkerClient
from shared.enums import AgentName
from shared.models.schemas import PlannerSubmissionResult
from shared.simulation.schemas import CustomObjectives, RandomizationStrategy
from worker_heavy.utils.file_validation import (
    validate_benchmark_definition_yaml,
    validate_node_output,
)


def validate_benchmark_planner_handoff_payload(
    *,
    artifacts: dict[str, str],
    session_id: str | None = None,
    custom_objectives: CustomObjectives | None = None,
    submission: PlannerSubmissionResult | None = None,
    submission_error: str | None = None,
    plan_output: Any | None = None,
    require_submission: bool = False,
    require_structured_plan: bool = False,
) -> list[str]:
    """Validate a benchmark planner handoff payload across entry and exit paths."""
    errors: list[str] = []

    if artifacts:
        is_valid, structural_errors = validate_node_output(
            AgentName.BENCHMARK_PLANNER, artifacts
        )
        if not is_valid:
            errors.extend([f"planner_structural: {msg}" for msg in structural_errors])

    objectives_text = artifacts.get("benchmark_definition.yaml")
    if objectives_text:
        try:
            is_valid, objectives_or_errors = validate_benchmark_definition_yaml(
                objectives_text,
                session_id=session_id,
            )
            if not is_valid:
                errors.extend(
                    [f"planner_semantic: {message}" for message in objectives_or_errors]
                )
                objectives = None
            else:
                objectives = objectives_or_errors
            if custom_objectives and objectives is not None:
                if custom_objectives.max_unit_cost is not None:
                    observed = objectives.constraints.max_unit_cost
                    expected = custom_objectives.max_unit_cost
                    if abs(observed - expected) > 1e-6:
                        errors.append(
                            "planner_semantic: objectives.constraints.max_unit_cost "
                            f"({observed}) does not match custom objective ({expected})"
                        )
                if custom_objectives.max_weight is not None:
                    observed = objectives.constraints.max_weight_g
                    expected = custom_objectives.max_weight
                    if abs(observed - expected) > 1e-6:
                        errors.append(
                            "planner_semantic: objectives.constraints.max_weight_g "
                            f"({observed}) does not match custom objective ({expected})"
                        )
                if custom_objectives.target_quantity is not None:
                    observed = objectives.constraints.target_quantity
                    expected = custom_objectives.target_quantity
                    if observed != expected:
                        errors.append(
                            "planner_semantic: objectives.constraints.target_quantity "
                            f"({observed}) does not match custom objective ({expected})"
                        )
        except Exception as exc:
            errors.append(
                f"planner_semantic: Failed to parse benchmark_definition.yaml ({exc})"
            )

    if require_submission:
        if submission is not None:
            if not submission.ok or submission.status != "submitted":
                errors.append(
                    "planner_submission: submit_plan() did not return submitted"
                )
            if submission.node_type != AgentName.BENCHMARK_PLANNER:
                errors.append(
                    "planner_submission: submission node_type is not benchmark_planner"
                )
            if submission.errors:
                errors.extend(
                    [f"planner_submission: {msg}" for msg in submission.errors]
                )
        elif submission_error:
            errors.append(f"planner_submission: {submission_error}")

    if require_structured_plan:
        if plan_output is None:
            errors.append(
                "planner_execution: missing structured planner output. "
                "Retry planner and call submit_plan() before handoff."
            )
        elif isinstance(plan_output, dict):
            is_plan_valid = False
            try:
                RandomizationStrategy.model_validate(plan_output)
                is_plan_valid = True
            except Exception:
                pass
            if not is_plan_valid:
                errors.append(
                    "planner_execution: planner output does not match RandomizationStrategy"
                )
        elif not isinstance(plan_output, RandomizationStrategy):
            errors.append(
                "planner_execution: planner output has unexpected type; "
                "expected RandomizationStrategy"
            )

    return errors


async def validate_benchmark_planner_handoff_artifacts(
    client: WorkerClient,
    *,
    custom_objectives: CustomObjectives | None = None,
    submission: PlannerSubmissionResult | None = None,
    submission_error: str | None = None,
    plan_output: Any | None = None,
    require_submission: bool = False,
    require_structured_plan: bool = False,
) -> list[str]:
    """Validate the planner-to-coder handoff package from the worker session."""
    errors: list[str] = []
    files_to_check = (
        "plan.md",
        "todo.md",
        "benchmark_definition.yaml",
        "assembly_definition.yaml",
    )
    artifacts: dict[str, str] = {}

    for rel_path in files_to_check:
        if not await client.exists(rel_path):
            errors.append(f"Missing planner artifact: {rel_path}")
            continue
        artifacts[rel_path] = await client.read_file(rel_path)

    errors.extend(
        validate_benchmark_planner_handoff_payload(
            artifacts=artifacts,
            session_id=client.session_id,
            custom_objectives=custom_objectives,
            submission=submission,
            submission_error=submission_error,
            plan_output=plan_output,
            require_submission=require_submission,
            require_structured_plan=require_structured_plan,
        )
    )
    return errors


def extract_custom_objectives_from_state(state: Any) -> CustomObjectives | None:
    """Best-effort extractor for custom objectives from a graph state or mapping."""
    session = (
        state.get("session")
        if isinstance(state, dict)
        else getattr(state, "session", None)
    )
    raw = (
        session.get("custom_objectives")
        if isinstance(session, dict)
        else getattr(session, "custom_objectives", None)
    )
    if raw is None or isinstance(raw, CustomObjectives):
        return raw
    try:
        return CustomObjectives.model_validate(raw)
    except Exception:
        return None
