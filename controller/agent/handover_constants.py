from __future__ import annotations

from shared.script_contracts import BENCHMARK_SCRIPT_PATH, SOLUTION_SCRIPT_PATH

ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS: tuple[str, ...] = (
    "benchmark_assembly_definition.yaml",
    BENCHMARK_SCRIPT_PATH,
)

ENGINEER_BENCHMARK_SOURCE_ARTIFACTS: tuple[str, ...] = (SOLUTION_SCRIPT_PATH,)

BENCHMARK_PLANNER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    "plan.md",
    "todo.md",
    "benchmark_definition.yaml",
    "benchmark_assembly_definition.yaml",
)

BENCHMARK_CODER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    *BENCHMARK_PLANNER_HANDOFF_ARTIFACTS,
    BENCHMARK_SCRIPT_PATH,
)

ENGINEERING_EXECUTION_REVIEWER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    SOLUTION_SCRIPT_PATH,
    BENCHMARK_SCRIPT_PATH,
    "benchmark_assembly_definition.yaml",
    "validation_results.json",
    "simulation_result.json",
)

ELECTRONICS_REVIEWER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    SOLUTION_SCRIPT_PATH,
    BENCHMARK_SCRIPT_PATH,
    "benchmark_assembly_definition.yaml",
    "validation_results.json",
    "simulation_result.json",
)

ENGINEER_PLANNER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    "plan.md",
    "todo.md",
    "benchmark_definition.yaml",
    "assembly_definition.yaml",
    *ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS,
)

BENCHMARK_PLAN_REVIEW_MANIFEST = ".manifests/benchmark_plan_review_manifest.json"
ENGINEERING_PLAN_REVIEW_MANIFEST = ".manifests/engineering_plan_review_manifest.json"
BENCHMARK_REVIEW_MANIFEST = ".manifests/benchmark_review_manifest.json"
ENGINEERING_EXECUTION_HANDOFF_MANIFEST = (
    ".manifests/engineering_execution_handoff_manifest.json"
)
ELECTRONICS_REVIEW_MANIFEST = ".manifests/electronics_review_manifest.json"

BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK = "benchmark_plan_reviewer_handover"
BENCHMARK_REVIEWER_HANDOVER_CHECK = "benchmark_reviewer_handover"
BENCHMARK_CODER_HANDOVER_CHECK = "benchmark_coder_handover"
ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK = "engineer_plan_reviewer_handover"
ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK = "engineer_execution_reviewer_handover"
ELECTRONICS_REVIEWER_HANDOVER_CHECK = "electronics_reviewer_handover"
ENGINEER_BENCHMARK_HANDOVER_CHECK = "engineer_benchmark_handover"

SCHEMA_BACKED_HANDOFF_PATHS: tuple[str, ...] = (
    "plan.md",
    "todo.md",
    "plan_refusal.md",
    "benchmark_definition.yaml",
    "assembly_definition.yaml",
    "benchmark_assembly_definition.yaml",
    "benchmark_plan_evidence_script.py",
    "benchmark_plan_technical_drawing_script.py",
    "solution_plan_evidence_script.py",
    "solution_plan_technical_drawing_script.py",
    "renders/benchmark_renders/render_manifest.json",
    "renders/engineer_renders/render_manifest.json",
    BENCHMARK_SCRIPT_PATH,
    SOLUTION_SCRIPT_PATH,
    "validation_results.json",
    "simulation_result.json",
    BENCHMARK_PLAN_REVIEW_MANIFEST,
    ENGINEERING_PLAN_REVIEW_MANIFEST,
    BENCHMARK_REVIEW_MANIFEST,
    ENGINEERING_EXECUTION_HANDOFF_MANIFEST,
    ELECTRONICS_REVIEW_MANIFEST,
)


__all__ = [
    "BENCHMARK_CODER_HANDOVER_CHECK",
    "BENCHMARK_CODER_HANDOFF_ARTIFACTS",
    "BENCHMARK_PLANNER_HANDOFF_ARTIFACTS",
    "BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK",
    "BENCHMARK_PLAN_REVIEW_MANIFEST",
    "BENCHMARK_REVIEWER_HANDOVER_CHECK",
    "BENCHMARK_REVIEW_MANIFEST",
    "ELECTRONICS_REVIEWER_HANDOVER_CHECK",
    "ELECTRONICS_REVIEW_MANIFEST",
    "ELECTRONICS_REVIEWER_HANDOFF_ARTIFACTS",
    "ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS",
    "ENGINEER_BENCHMARK_SOURCE_ARTIFACTS",
    "ENGINEER_BENCHMARK_HANDOVER_CHECK",
    "ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK",
    "ENGINEERING_EXECUTION_REVIEWER_HANDOFF_ARTIFACTS",
    "ENGINEER_PLANNER_HANDOFF_ARTIFACTS",
    "ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK",
    "ENGINEERING_EXECUTION_HANDOFF_MANIFEST",
    "ENGINEERING_PLAN_REVIEW_MANIFEST",
    "SCHEMA_BACKED_HANDOFF_PATHS",
]
