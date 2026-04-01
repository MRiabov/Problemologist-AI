# Failure Modes

## Table of Contents

1. [Geometry and placement](#geometry-and-placement)
2. [Contract and schema](#contract-and-schema)
3. [Manufacturability and cost](#manufacturability-and-cost)
4. [Simulation and robustness](#simulation-and-robustness)
5. [Evidence and review](#evidence-and-review)
6. [Refusal and handoff](#refusal-and-handoff)

## Geometry and placement

Signals:

- Intersections
- Out-of-bounds parts
- A path that looks correct in sketch space but not in final placement
- Missing support continuity between adjacent surfaces

Likely causes:

- Wrong datum
- Wrong rotation
- Gaps between handoff surfaces
- The final envelope was not checked after placement

First fix:

- Inspect the final placed geometry and fix the narrowest placement error.

## Contract and schema

Signals:

- Missing or malformed handoff files
- Wrong top-level labels
- `result` not bound correctly
- Import-time side effects
- Wrote benchmark-owned files from the engineer side

Likely causes:

- The script contract was not kept import-safe
- Ownership boundaries were ignored
- A fallback was invented instead of fixing the source

First fix:

- Restore the file contract and ownership boundaries before changing the design.

## Manufacturability and cost

Signals:

- Cost over budget
- Weight over budget
- Unsupported stock assumptions
- Unclear or impossible machining access
- Material choice does not match the process

Likely causes:

- Too many unique parts
- Excessive volume
- Wrong process family
- Geometry that is easy to model but hard to make

First fix:

- Simplify the mechanism or switch to a more realistic process/material combination.

## Simulation and robustness

Signals:

- Goal reached only on the nominal seed
- Jitter-sensitive failure
- Overdependence on a precise start position
- Unstable motion or flakiness

Likely causes:

- Capture margins are too small
- Contact geometry is too delicate
- The mechanism assumes exact timing
- The design has unnecessary degrees of freedom

First fix:

- Increase tolerance to runtime variation and remove fragile dependency points.

## Evidence and review

Signals:

- Reviewer rejects a plan or implementation that looked plausible at a glance
- Render evidence was ignored
- Text summaries contradicted the actual media
- A mixed review was treated as a blanket rejection or a blanket approval

Likely causes:

- The wrong evidence was used
- Media inspection was skipped
- Valid reviewer items were mixed with non-applicable requests

First fix:

- Inspect the actual media and separate valid checklist items from noise.

## Refusal and handoff

Signals:

- The plan cannot be implemented as written
- The handoff contradicts itself
- Budgets or objective geometry make the task impossible

Likely causes:

- The planner handoff is internally inconsistent
- Required benchmark-owned facts are missing
- The task requires a different mechanism family than the handoff allows

First fix:

- Write `plan_refusal.md` with concrete evidence and stop pretending the plan is salvageable.
