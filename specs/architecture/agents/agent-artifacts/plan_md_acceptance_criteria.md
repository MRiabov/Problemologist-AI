# plan.md Legacy Compatibility Criteria

## Role of the File

`plan.md` is a compatibility alias for historical seed bundles and replay artifacts.
Newly materialized workspaces should use `benchmark_plan.md` or `engineering_plan.md` instead.

## Hard Requirements

- The file is only acceptable in compatibility or replay contexts.
- The stage-specific contract remains identifiable from the surrounding benchmark or engineering artifacts.
- The legacy file does not become the canonical authored filename for new workspaces.
- The contents still agree with the benchmark or engineering contract they mirror.

## Quality Criteria

- Historical rows remain readable without reauthoring the corpus.
- Seed and replay tooling can map the legacy filename to the appropriate stage-specific contract.
- The doc is treated as transitional, not the preferred path.

## Reviewer Look-Fors

- New starter bundles still trying to author `plan.md`.
- The legacy file is used where a stage-specific plan filename is required.
- The contents drift from the stage-specific contract.

## Cross-References

- `specs/architecture/agents/agent-artifacts/benchmark_plan_md_acceptance_criteria.md`
- `specs/architecture/agents/agent-artifacts/engineering_plan_md_acceptance_criteria.md`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
