from __future__ import annotations

ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS: tuple[str, ...] = (
    "benchmark_assembly_definition.yaml",
)

BENCHMARK_PLANNER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    "plan.md",
    "todo.md",
    "benchmark_definition.yaml",
    "benchmark_assembly_definition.yaml",
)

REVIEWER_HANDOFF_ARTIFACTS: tuple[str, ...] = (
    "script.py",
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
ENGINEERING_EXECUTION_REVIEW_MANIFEST = (
    ".manifests/engineering_execution_review_manifest.json"
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
    "validation_results.json",
    "simulation_result.json",
    BENCHMARK_PLAN_REVIEW_MANIFEST,
    ENGINEERING_PLAN_REVIEW_MANIFEST,
    BENCHMARK_REVIEW_MANIFEST,
    ENGINEERING_EXECUTION_REVIEW_MANIFEST,
    ELECTRONICS_REVIEW_MANIFEST,
)


__all__ = [
    "BENCHMARK_CODER_HANDOVER_CHECK",
    "BENCHMARK_PLANNER_HANDOFF_ARTIFACTS",
    "BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK",
    "BENCHMARK_PLAN_REVIEW_MANIFEST",
    "BENCHMARK_REVIEWER_HANDOVER_CHECK",
    "BENCHMARK_REVIEW_MANIFEST",
    "ELECTRONICS_REVIEWER_HANDOVER_CHECK",
    "ELECTRONICS_REVIEW_MANIFEST",
    "ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS",
    "ENGINEER_BENCHMARK_HANDOVER_CHECK",
    "ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK",
    "ENGINEER_PLANNER_HANDOFF_ARTIFACTS",
    "ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK",
    "ENGINEERING_EXECUTION_REVIEW_MANIFEST",
    "ENGINEERING_PLAN_REVIEW_MANIFEST",
    "SCHEMA_BACKED_HANDOFF_PATHS",
    "REVIEWER_HANDOFF_ARTIFACTS",
]
