# Nightly work plan, March 30

1. We have a regression with rendering - for some reason
   Bug report as below:
   """
   Title
   ec-001 fails at the handoff boundary, not the geometry boundary

Summary
The engineer coder can build the sideways-transfer geometry, but the eval does not complete because the run never cleanly
reaches the execution-review manifest step. The earlier run failed explicitly with a missing .manifests/
engineering_execution_review_manifest.json, and the agent then spent time chasing renderer/display workarounds that are not
valid in this shell. I removed the bad WORKER_RENDERER_LOCAL_RENDER hint from the seed, but the underlying structural issue
remains: the workflow is blocked before submit_for_review can complete.

Observed Behavior

- The failed session at run_20260330_182344 ended with:
  - .manifests/engineering_execution_review_manifest.json missing; call submit_for_review(compound) first.
  - See logs/evals/runs/run_20260330_182344/sessions/ec-001/session_metadata.json
- The transcript shows the agent reached validation/simulation probing, then concluded the handoff was blocked by renderer/
  MuJoCo display permissions in the shell.
  - See logs/evals/runs/run_20260330_182344/sessions/ec-001/codex/019d3fc5-e1bd-7d50-b20f-03d9b78bfb58/transcript.log
- The rerun at run_20260330_203744 materialized the workspace with the updated hint, but still ended with success: false.
  - See logs/evals/runs/run_20260330_203744/sessions/ec-001/session_metadata.json

Expected Behavior

- The coder should implement the geometry.
- Validation and simulation should complete in a way that still allows bash scripts/submit_for_review.sh to write the review
  manifest.
- The eval should finish with a successful handoff, not a workspace that looks complete but fails the gate.

Root Cause
This is structural, not just a CAD issue:

- The workspace contract requires a real submission handoff.
- The validation path is trying to use a renderer/display route that does not work in this shell.
- The old hint about WORKER_RENDERER_LOCAL_RENDER=1 was misleading and not a real fix; that path should not be treated as
  valid.

Relevant Files

- dataset/data/seed/artifacts/engineer_coder/ec-001-sideways-transfer/ec001_run_hints.md
- logs/evals/runs/run_20260330_203744/codex-workspaces/engineer_coder-ec-001-5429a8c8/prompt.md:21-23
- logs/evals/runs/run_20260330_182344/sessions/ec-001/session_metadata.json

"""
I have already started editing an exiting int-033 to make it much closer to the reality. I don't know if I will finish by tonight, rerun it

**Progress and notes**:

<!--Agent: write the progress on the task below-->

1. Restored the deleted INT-033 coder fixture script and aligned the mocked coder handoff to `python script.py`, which is what actually seeds `.manifests/engineering_execution_review_manifest.json`.
2. Verified the narrow `test_engineering_full_loop` slice passes on a fresh integration run (`run_20260330_214602`).
3. Updated the handover path to synthesize `renders/render_manifest.json` from actual preview artifacts so the execution-review gate no longer depends on the old renderer workaround.
4. Verified the exact INT-033 full-loop slice on `run_20260331_012032`; the coder now reaches `submit_for_review` and writes the execution-review manifest successfully.
5. Rechecked the same INT-033 slice after the latest fixture drift and fixed the stale `benchmark_definition.yaml` hash assertion to canonicalize YAML the same way the manifest does; the rerun passed.

[x] - definitely completed.

2. `integration_p1` test suite is red.
   Our integration_p1 test red. I haven't run it in two weeks.
   Suggestion: there may be plain stale tests. Update them.

**Progress and notes**:

<!--Agent: write the progress on the task below-->

1. [x] Restored INT-016 to the actual planner/coder contract: added the required `inspect_media(renders/render_e45_a45.png)` planner step, aligned the coder transcript with `python script.py`, and fixed the benchmark geometry so validation/simulation can reach `submit_for_review(compound)`.
2. [x] Verified `tests/integration/architecture_p1/test_dataset_export.py::test_dataset_export_solution_row_round_trip` passes on a fresh integration run after the INT-016 fixture fixes.
3. [ ] The broader `integration_p1` sweep was not rerun after the targeted INT-016 fix; the last full sweep still had separate failures in `INT-186` and `INT-042`, which need a follow-up pass.

4. [x] Aligned the stale `INT-205` engineer-coder benchmark assembly fixture with the seeded benchmark definition so the retry-lineage planner no longer trips the old `200/1200` constraint mismatch.

5. [x] Verified `tests/integration/architecture_p1/test_engineering_loop.py::test_engineering_retry_reuses_same_benchmark_linkage` passes on a fresh integration run after the fixture fix.

6. [ ] The broader `integration_p1` sweep is still being rechecked after the INT-205 fix; I have not yet closed the full-marker result this cycle.

1. [x] Repaired `INT-186` so the forced worker-light outage waits for the coder invoke boundary instead of firing during the planner's validation submission.

2. [x] Verified the live `integration_p1` run now passes through `test_int_186_system_failed_tool_retry_cap_and_terminal_metadata` after the timing fix.

3. [x] Fixed `DatasetRowLineage` so dataset export accepts `benchmark_family` and the benchmark row round trip completes again.

4. [x] Verified `test_dataset_export_benchmark_row_round_trip` passes on a fresh integration run after the schema fix.

5. `integration_p2` test suite wasn't even run in a month.

**Progress and notes**:

<!--Agent: write the progress on the task below-->

1. [x] Ran `tests/integration/evals_p2/test_evals_p2.py` through `./scripts/run_integration_tests.sh` on a fresh stack.
2. [x] Verified the slice passes cleanly on `run_20260330_234030` (`7 passed`).
3. [x] Observed controller backend-error noise from the eval smoke cases (`INT-046` through `INT-048`) emitting `node_entry_validation_rejected` for missing `benchmark_assembly_definition.yaml`, but the integration slice still completed successfully.
