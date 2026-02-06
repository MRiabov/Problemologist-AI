---
work_package_id: "WP05"
title: "Domain Utils (Worker Side)"
lane: "planned"
dependencies: ["WP02"]
subtasks: ["T022", "T023", "T024", "T025", "T026"]
---

## WP05: Domain Utils (Worker Side)

**Goal**: Implement the "OS-level" utilities that agents will import and usage.

## Context

Agents don't write raw API calls or raw MuJoCo XML. They import `utils` which abstracts the complexity.
These utils run *inside* the Worker container (invoked by `runtime`).

## Subtasks

### T022: Create Utils Package

Location: `src/worker/utils/`

- `__init__.py`: Export public functions.
- `simulation.py`: Logic for simulation.
- `validation.py`: Logic for pricing/checks.
- `submission.py`: Logic for review.

### T023: Implement `validate_and_price`

File: `src/worker/utils/validation.py`
Signature: `validate_and_price(component: Part|Compound) -> dict`

Logic (Stub):

- Check input type (must be `build123d.Part` or `Compound`).
- Calculate bounding box.
- (Mock) Return `{"valid": True, "cost": 10.5}`.

### T024: Implement `simulate`

File: `src/worker/utils/simulation.py`
Signature: `simulate(component: Compound) -> SimulationResult`

Logic:

1. **Git Commit**: Call `git add . && git commit -m "Snapshot before simulation"`.
2. **Trigger Workflow**: Call Controller API `POST /simulation/run` (which starts a Temporal `SimulationWorkflow`).
3. **Wait/Poll**: Wait for the result from the Controller.
4. **Result**: The result includes S3 URLs for video and summary markdown.
5. **Return**: Markdown summary (e.g. "Simulation Passed. Video: https://...").

### T025: Implement `submit_for_review`

File: `src/worker/utils/submission.py`
Logic:

- Mark the current state as "Ready".
- Maybe write a `REVIEW_REQUEST.md` file.

### T037: Implement 24-view preredering

- Implement logic to generate 24 renders (8 angles x 3 levels) for environment assets.
- Save results to the `/renders/` folder (routed to S3).
- Ensure this is triggered during benchmark finalization for handover.

### T026: PYTHONPATH Integration

- Ensure `src.worker.utils` is importable as `utils` inside the `runtime` execution.
- Update `runtime/executor.py` (from WP02) to add `src/worker` (or wherever utils lives) to `PYTHONPATH` before execution.

## Verification

1. Create a script `test_utils.py`:

   ```python
   from utils import simulate
   from build123d import Box
   res = simulate(Box(1))
   print(res)
   ```

2. Run it via `runtime/execute` API.
3. Expect success output.
