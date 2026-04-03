# Reviewer Manifest Acceptance Criteria

Use this reference for stage-scoped reviewer seeds.

## Required Manifests

- `.manifests/benchmark_review_manifest.json`
- `.manifests/engineering_plan_review_manifest.json`
- `.manifests/engineering_execution_handoff_manifest.json`
- `.manifests/electronics_review_manifest.json`

Note: older docs in this repo may still use the legacy `engineering_execution_review_manifest.json` wording. The live runtime contract uses the handoff name above.

## Acceptance Criteria

- Reviewer stages fail closed when the manifest is missing, stale, or schema-invalid.
- The manifest must match the stage being evaluated.
- The manifest must point at the latest revision only.
- Comments YAML must be factual, evidence-based, and tied to the current package.
- Review decisions must not rely on filename inspection alone when media evidence exists.

## Useful Fields

- `latest_revision_verified`
- `review_manifest_revision`
- `validation_success`
- `simulation_success`
- `goal_reached`
- `solvability_summary`
- `attachment_policy_summary`
- `render_count`
- `inspected_render_count`
- `visual_inspection_min_images`
- `visual_inspection_satisfied`
- `visual_evidence_checked`
- `deterministic_error_count`
- `deterministic_refusal_reason`
- `dynamic_evidence_checked` when moving benchmark fixtures exist
