# Motion Trajectory Review

Use this reference whenever a plan or implementation includes a motion or payload trajectory claim.

## Review Rule

- Motion and payload trajectories must always be described and justified with formulas.
- Reject prose-only trajectories.
- Do not infer the path from mechanism prose when the handoff is supposed to be reviewable.

## Plan Review Checks

- Verify the trajectory is reconstructable from the handoff artifacts.
- Confirm the anchor sequence, contact order, and tolerance bands are explicit.
- Check that the first anchor is build-zone valid and the terminal proof reaches the goal zone or goal-contact condition.
- Verify the calculations back every binding numeric claim in the motion forecast.
- Reject missing or contradictory motion forecasts.

## Execution Review Checks

- Verify the implemented motion matches the approved trajectory contract.
- Check that validation and simulation evidence support the same anchor sequence and contact order.
- Reject runtime evidence that contradicts the approved motion or leaves the path ambiguous.

## Acceptance Standard

- If a reviewer has to infer the motion from prose, the handoff is incomplete.
- If a required input is missing, reject the handoff rather than estimating it.
