# Integration Test Review - March 6, P1 Coverage

## Scope

Review of P1 integration tests against `@specs/integration-tests.md` for spec drift and implementation completeness.

## Findings

1. Missing coverage for WP3 (Fluids & Electronics) P1 workflows.

- Severity: High
- Why this matters: Critical architectural boundaries for the latest Work Package 3 (Fluids/Electronics) are not verified in an integrated environment, increasing regression risk for multi-agent loops and conflict resolution.
- Evidence:
  - `INT-131 through INT-137, INT-140, INT-141` are absent from `tests/integration/`.
  - `grep_search` across `tests/integration/` returned 0 matches for these IDs.
- Notes: These IDs cover the full `planner -> engineer -> reviewer` loops for fluids and `mech -> elec -> reviewer` iteration cycles.

Human: Done.

2. Benchmark-to-engineer handoff package (INT-032) is underspecified.

- Severity: Medium
- Why this matters: The test passes if any renders exist, but fails to verify that the engineer receives the full metadata bundle (DOFs, jitter, etc.) required for design stability.
- Evidence:
  - `tests/integration/architecture_p1/test_handover.py:84`
- Notes: Verification currently stops at `renders/` directory existence and `benchmark_definition.yaml` presence.

Human: had singificant logic there.

<!--3. Reviewer evidence completeness (INT-034) lacks strict field validation.
- Severity: Medium
- Why this matters: The spec requires `images_viewed` and `files_checked` as evidence, but the test only checks for a non-empty `comments` field in the frontmatter.
- Evidence:
  - `tests/integration/architecture_p1/test_reviewer_evidence.py:165`
- Notes: Current implementation uses `ReviewFrontmatter.model_validate`, which may pass even if specific evidence fields are missing or generic.-->

4. Skill safety toggle (INT-065) is missing functional integration.

- Severity: Medium
- Why this matters: The logic is only verified at the schema/unit level, but the actual block/revert behavior in a live agent session is untested.
- Evidence:
  - `tests/integration/architecture_p1/test_int_064_to_069.py:65`
- Notes: Test is a stub verifying only that the `SkillEditEvent` can be instantiated.

5. Materials config enforcement (INT-035) uses a weak success path.

- Severity: Low
- Why this matters: The test verifies that the system "handles" a bad material, but does not strictly assert that the pipeline rejected the run or reverted the session as mandated.
- Evidence:
  - `tests/integration/architecture_p1/test_manufacturing.py:116`
- Notes: Minimal assertion ensures the episode doesn't crash but doesn't verify the "fail closed" contract.

## Suggested Remediation Order

1. Implement `tests/integration/architecture_p1/test_physics_fluids_full.py` to cover `INT-131, 133, 134, 135, 138, 139`.
2. Implement `tests/integration/architecture_p1/test_electronics_full.py` to cover `INT-132, 136, 137, 140, 141`.
3. Strengthen `test_handover.py` (INT-032) to assert against moving-parts DOFs and jitter metadata.
4. Update `test_reviewer_evidence.py` (INT-034) to strictly require `images_viewed` and `files_checked` in parsed frontmatter.
5. Upgrade the INT-065 stub to a functional integration test using a scripted agent session.
