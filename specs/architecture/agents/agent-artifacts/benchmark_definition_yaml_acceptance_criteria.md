# benchmark_definition.yaml Acceptance Criteria

## Role of the File

`benchmark_definition.yaml` is the benchmark-owned contract for task geometry, objective zones, randomization, environment assumptions, and customer caps.
It is worth being a first-class agent artifact because downstream roles, validators, and eval seeds all need the same exact geometry and bounds, not a paraphrase.

## Hard Requirements

- The file schema-validates before execution continues.
- Goal-zone, forbid-zone, and build-zone geometry are exact and internally consistent.
- The moved object has a top-level `start_position`, stable labels, and a `material_id` that resolves to a known material from `manufacturing_config.yaml`.
- Static and runtime randomization ranges are exact, bounded, and materialized at seed time.
- Benchmark caps, estimate fields, and other numeric fields are exact, not deferred to later inference.
- Any benchmark-owned moving fixture is declared explicitly and remains motion-visible to downstream roles.
- The file contains no placeholders, invented identifiers, or unstated benchmark fixtures.

## Quality Criteria

- The contract creates a real challenge without hidden contradictions or degenerate geometry.
- The randomization is bounded enough to remain reproducible and broad enough to exercise robustness.
- Exact identifiers are repeated across the handoff package so later roles do not have to guess names or counts.
- The geometry and cap data are legible enough that reviewers can reason about solvability without reconstructing intent from scratch.

## Reviewer Look-Fors

- Goal, forbid, and build geometry intersect the moved object at spawn or after stated randomization.
- `moved_object.material_id` is missing, empty, or unknown to `manufacturing_config.yaml`.
- Runtime or static randomization pushes the object out of bounds or into an impossible start state.
- Hidden benchmark motion or unsupported fixture declarations appear in the file.
- Cap, estimate, or label drift appears across the handoff package.

## Cross-References

- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/definitions-of-success-and-failure.md`
- `specs/architecture/evals-architecture.md`
