# Benchmark Patterns

## Table of Contents

1. [How to choose](#how-to-choose)
2. [Passive benchmark geometry](#passive-benchmark-geometry)
3. [Moving benchmark fixtures](#moving-benchmark-fixtures)
4. [Convergence heuristics](#convergence-heuristics)

## How to choose

Choose the smallest benchmark geometry family that can satisfy the approved plan under runtime jitter.

- If gravity and shape alone are enough, use passive geometry.
- If the benchmark explicitly needs motion, add only the minimum fixture motion required by the handoff.
- If the plan depends on exact part identity, preserve the exact catalog part instance.
- If the plan includes objective overlays or moving benchmark-owned fixtures, keep them reviewable and explicit rather than clever.

## Passive benchmark geometry

Common forms:

- Wall
- Ramp
- Funnel
- Chute
- Deflector
- Capture pocket
- Guide rail

Drafting cues:

- Keep the transport path continuous.
- Check the final placed envelope, not just the sketch or local part.
- Leave enough lateral margin for runtime jitter.
- Avoid unsupported gaps, tiny ledges, or exact-seed-only clearance.
- Base every wall, gap, capture span, and offset on benchmark-owned geometry or an explicit formula. Never eyeball a clearance when the inputs are already known.

## Moving benchmark fixtures

Use motion only when the approved benchmark explicitly declares it.

Common forms:

- Sliding gate
- Rotating blocker
- Sweeper arm
- Latch
- Resettable trap

Drafting cues:

- Keep the motion contract explicit.
- Preserve the declared axis, path, travel range, and trigger behavior.
- Make the motion visible in evidence and reviewable from the handoff.
- Avoid extra DOFs that do not improve the benchmark.
- Derive the resting pose and travel limits from the declared joint or axis, not from an arbitrary placement guess.

## Convergence heuristics

- Start from the simplest plausible geometry and only add complexity after a concrete failure explains why.
- If the first simulation goes the wrong way, treat it as a directionality or contract bug before widening the geometry.
- Prefer robust margins over exact geometric coincidence.
- If the approved benchmark plan already pins down the exact geometry, labels, or inventory, copy that contract forward verbatim into `benchmark_script.py` and spend effort on the build123d translation, not on re-deciding the benchmark.
- Keep benchmark-owned objects legible for downstream engineer intake.
