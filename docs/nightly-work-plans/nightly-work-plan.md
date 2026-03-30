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

\[x\] - definitely completed?

2. `integration_p1` test suite is red.
   Our integration_p1 test red. I haven't run it in two weeks.
   Suggestion: there may be plain stale tests. Update them.

**Progress and notes**:

<!--Agent: write the progress on the task below-->

1. ...

2. ...

3. `integration_p2` test suite wasn't even run in a month.
