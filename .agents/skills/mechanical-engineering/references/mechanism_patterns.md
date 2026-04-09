# Mechanical Mechanism Patterns

## Contents

1. Choosing a mechanism family
2. Passive transfer patterns
3. Failure patterns to reject
4. Practical geometry heuristics
5. Simulation-first iteration loop
6. When to escalate beyond passive transfer

## 1. Choosing a mechanism family

Start with the minimum mechanism that can satisfy the objective.

- If gravity and existing height difference can solve the task, prefer a passive guide surface.
- If the object must be captured while changing direction, use a chute, channel, or funnel rather than a flat plate plus hope.
- If the task needs timed or powered motion, then add the smallest justified active mechanism.
- If the task can be solved statically, keep `dofs: []`.

## 2. Passive Transfer Patterns

### Continuous runway

Use when the payload starts above the target path and only needs directed travel.

- Provide one continuous support path from spawn support to goal support.
- Ensure the travel direction has a real downhill component.
- Add side retention if jitter or bounce could eject the object.
- Verify the object can physically touch the runway at the declared spawn height before tuning capture details.
- If spawn sits above the runway, lower the first support or raise the path before widening the pocket.

### Chute or channel

Use when the object must stay captured during lateral travel.

- Build the floor first, then add walls or rails that keep the object inside the transport corridor.
- Prefer simple rectangular walls when they are enough.
- Rotate or incline the transport surface only when the slope itself is required.

### Funnel or deflector

Use when the object arrives with spread in position and needs to be collected or redirected.

- Widen the intake region where jitter enters.
- Narrow only after the object is already constrained.
- Deflectors should resolve a specific trajectory risk, not decorate the design.

### Pocket or cradle

Use when the object must rest safely before the next motion step.

- Support the object at its actual start position.
- Do not leave the object effectively in free space and rely on simulation settling.
- If the object never reaches the path, fix first-contact height before increasing pocket depth or width.

## 3. Failure Patterns To Reject

Reject these drafts before simulation:

- Two flat platforms with a gap and no transport surface between them.
- A passive-transfer design whose lateral motion depends on bounce, spawn jitter, or collision luck.
- Geometry that changes support heights but never creates a sloped contact surface.
- A design where the only authored parts are a start pedestal, some rails, and the payload.
- Added DOFs or actuators that are not required by the task.
- Rotated or exotic primitives chosen only because they look mechanical, when a simpler guide surface would work.

## 4. Practical Geometry Heuristics

### Transport first

Before writing the first full draft, answer:

- What exact child part creates directed motion?
- Where does the object first contact it?
- How does that part keep the object moving toward the goal instead of leaving by chance?

If those answers are unclear, the mechanism is underspecified.

### Capture width

Size the support path from the payload envelope, not only from nominal geometry.

- Include object radius or body size.
- Include runtime jitter.
- Include enough margin that small contact errors do not eject the object immediately.

### Dimensioning contract

Derive every dimension from named source values or formulas.

- Use sums and differences of known dimensions for spans, offsets, wall thicknesses, and clearances.
- Pull material or manufacturing constants from the workspace config or part specs, not from memory.
- If a required value is missing, stop and mark the design incomplete instead of guessing a plausible size.

### Joint-driven placement

Treat the joint frame or contact datum as the source of pose.

- Compute part locations from joint axes and mating features.
- Do not place parts by arbitrary world coordinates when the mechanism is built around a real connection point.
- If a placement looks right only in one camera view, re-derive it from the geometry before trusting it.

### Friction check

For passive ramps and chutes, compare the actual slope to the workspace friction coefficient before spending time on capture details.

- Use `theta > atan(mu_static)` as the first-order check.
- Pull `mu_static` from the workspace `manufacturing_config.yaml`, not from memory.
- If `tan(theta)` is below the coefficient, the object may stall even if the geometry validates.

### Primitive choice

Prefer the least risky primitive family that satisfies the contract.

- Use axis-aligned boxes for rails, walls, stops, and simple supports when possible.
- Use rotation where slope or directional redirection is mechanically required.
- Avoid adding cylinders, splines, or complex profiles unless they solve a real contact problem.

### Continuity

Check for discontinuities in support:

- sudden unsupported gaps
- surfaces that visually align but do not actually meet
- rails without a floor
- floor segments that do not produce net downhill travel

## 5. Simulation-First Iteration Loop

Use this loop when a passive-transfer draft validates but the motion still fails.

1. Judge the draft by whether the object stays captured and reaches the goal, not just whether the geometry is valid.
2. Inspect the first simulation frames or video immediately when direction is uncertain.
3. If the object drifts the wrong way, fix slope direction, handedness, or datum orientation before seam cleanup, cosmetics, or cost tuning.
4. Preserve one continuous transport corridor from capture to goal; a flat funnel feeding a sloped lane is a bad handoff.
5. Treat seed hints for direction, x-position, and corridor placement as hard constraints when they are explicit.
6. Change one variable at a time. Do not mix placement, clearance, slope, and capture geometry changes in the same iteration.
7. Treat validation as a prerequisite, not evidence that the mechanism works.
8. If the object never reaches the support surface, treat it as a contact-height bug before a capture-margin bug.

## 6. When To Escalate Beyond Passive Transfer

Escalate only when passive geometry cannot satisfy the benchmark contract.

- Add active motion if the task requires lifting, timing, reversal, or controlled force.
- Add joints only when there is a real bearing, motor, slider, or equivalent physical basis.
- For stress-sensitive or fluid tasks, switch to the corresponding mechanical references after the mechanism concept is established.
