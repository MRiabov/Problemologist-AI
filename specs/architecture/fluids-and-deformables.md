# Fluids, FEM, and Stress Validation

## Scope summary

- Primary focus: the architecture contract for fluid benchmarks, deformable-material simulation, and stress-aware validation.
- Defines what benchmark planners, engineers, reviewers, and the simulation stack must support when a task goes beyond rigid-body-only mechanics.
- Defines the stable product-facing contracts in `benchmark_definition.yaml`, `manufacturing_config.yaml`, and the simulation/review artifact surface.
- Use this file for fluids, deformables, and stress-validation capability planning and product behavior.
- Use [simulation-and-rendering.md](./simulation-and-rendering.md) for generic simulation semantics that apply to all physics modes.

This document is intentionally high-level and stable. It describes what the system must support and how the major pieces fit together. It does not try to mirror the current source tree.

## Objective

The system must support mechanical engineering tasks where rigid-body-only physics is not enough.

That means:

1. parts may need to bend or fail under load,
2. fluid containment and flow may be the main objective,
3. fluid and solid interaction may decide whether a design passes,
4. the benchmark and engineering loops must be able to reason about those behaviors without inventing their own ad hoc conventions.

The goal of this WP is not "nice to have realism". The goal is to make these phenomena first-class parts of the benchmark contract and the engineering validation loop.

## Product goals

1. Fluid benchmarks are a first-class benchmark family, not a special-case experiment.
2. Stress and breakage become explicit pass/fail criteria rather than hidden debug signals.
3. Material choice in manufacturing configuration directly affects physics behavior.
4. Reviewers receive artifacts that make fluid and stress failures understandable.
5. The system keeps a fast iteration path for expensive Genesis scenarios without confusing that with final sign-off.

## Non-goals

This document does not require:

1. chemically accurate fluid mixing,
2. phase change,
3. surface-tension-grade CFD accuracy,
4. thermal-structural coupling,
5. arbitrary fluid authoring from complex CAD volumes as the default authoring path,
6. production-grade numerical calibration suitable for external scientific claims.

The target is engineering-useful benchmarking and solution validation, not a general-purpose fluids research platform.

## Core architecture decision

### Backend split

We do not force one simulator to do every job.

The architecture is:

1. Genesis is the required backend for fluid simulation, deformables, and stress-aware runs.
2. MuJoCo remains valid for rigid-only tasks and for the fast static preview path.
3. Static validation preview stays intentionally cheaper than Genesis simulation.
4. Backend selection is part of the benchmark contract, not a hidden worker decision.

### Why the split exists

The split exists because the system needs two different qualities:

1. a capable backend for fluids and FEM,
2. a fast backend for rigid preview and cheap validation artifacts.

Trying to erase that distinction would either make validation too expensive or make fluid/FEM support fake.

### Product implication

The planner and engineer can rely on a simple rule:

1. if the task needs fluid behavior, deformables, or stress failure, it is a Genesis task,
2. if the task is rigid-only, MuJoCo remains acceptable,
3. preview images are not themselves proof that Genesis behavior was exercised.

## Stable artifact contracts

### `benchmark_definition.yaml`

This capability extends `benchmark_definition.yaml` with three architecture-level concepts:

1. physics backend selection,
2. fluid definitions,
3. fluid and stress objectives.

The required structure is:

```yaml
physics:
  backend: GENESIS
  fem_enabled: true
  compute_target: auto

fluids:
  - fluid_id: water
    properties:
      viscosity_cp: 1.0
      density_kg_m3: 1000.0
      surface_tension_n_m: 0.07
    initial_volume:
      type: sphere
      center: [0, 0, 3]
      radius: 0.6
    color: [0, 0, 200]

objectives:
  goal_zone:
    min: [5, 5, 5]
    max: [7, 7, 7]
  build_zone:
    min: [-10, -10, -10]
    max: [10, 10, 10]
  fluid_objectives:
    - type: FLUID_CONTAINMENT
      fluid_id: water
      containment_zone:
        min: [-5, -5, -5]
        max: [5, 5, 5]
      threshold: 0.95
      eval_at: END
    - type: FLOW_RATE
      fluid_id: water
      gate_plane_point: [0, 0, 0]
      gate_plane_normal: [0, 0, 1]
      target_rate_l_per_s: 1.0
      tolerance: 0.2
  stress_objectives:
    - type: max_stress
      part_label: gripper_pad
      max_von_mises_mpa: 8.0
```

### What `physics` means

`physics` is part of the benchmark specification, not merely runtime tuning.

It answers:

1. which backend is expected,
2. whether the task expects deformable/stress-aware behavior,
3. whether the episode is intended as a high-cost final run or a cheaper iteration path.

### What `fluids` means

`fluids` declares the fluid bodies that exist in the episode.

Each fluid definition must carry:

1. a stable `fluid_id`,
2. material-like properties,
3. an initial volume primitive,
4. a display color suitable for review artifacts.

### What `fluid_objectives` and `stress_objectives` mean

These objective families make fluid and stress behavior explicit parts of the benchmark contract.

They are not optional debug metrics. They are part of what determines success or failure.

### `manufacturing_config.yaml`

Material properties in `manufacturing_config.yaml` are part of the simulation contract.

For this capability, the critical additions are:

1. density,
2. Young's modulus,
3. Poisson's ratio,
4. yield stress,
5. ultimate stress,
6. material class.

This is the bridge between manufacturability/pricing choices and structural behavior.

### `assembly_definition.yaml`

This capability does not require a new standalone fluid section inside `assembly_definition.yaml`.

The main physical coupling comes from:

1. the benchmark definition in `benchmark_definition.yaml`,
2. the geometry built by the planner/engineer,
3. the material assignments carried by the assembly parts,
4. the electronics section when a fluid benchmark is also electromechanical.

That is intentional. The benchmark declares what fluid exists and what must happen. The assembly defines the geometry and material choices that make success or failure likely.

## Material and deformable behavior

### Material ownership

Engineer-owned manufactured parts must always have a valid material assignment when they are expected to participate in pricing or stress-aware simulation.

Benchmark-owned environment fixtures are different:

1. they must still be geometrically valid,
2. they may carry physics-relevant metadata,
3. they are not priced as manufactured outputs.

### FEM enablement

FEM must be activated in a way that is legible to planners and reviewers.

The architecture rule is:

1. the benchmark can explicitly request FEM through `physics.fem_enabled`,
2. material classes may also imply deformable treatment for soft or elastomeric parts,
3. reviewers must treat stress-aware tasks as materially different from rigid-only tasks.

### Material classes

At the architecture level, we only need three material families:

1. `rigid`
2. `soft`
3. `elastomer`

The exact constitutive models are an implementation detail. The stable contract is that those categories behave differently under load and are not interchangeable.

### Breakage

Part breakage is a first-class failure mode.

The rule is:

1. if stress exceeds the part's ultimate limit, the run fails,
2. the failure must identify which part failed,
3. the reviewer must be able to see where and why the failure happened,
4. a passing design must not quietly rely on structural failure as a hidden mechanism.

### Stress summaries

Stress-aware runs must produce structured summaries that are simple enough for agents and reviewers to reason about.

The stable summary surface is:

1. part label,
2. peak stress,
3. mean stress,
4. safety factor,
5. location of the peak,
6. utilization against the material limit.

These values are part of the simulation artifact contract because they allow the review loop to explain structural failure without parsing raw solver output.

## Fluid behavior contract

### Fluid definition

Fluid authoring must stay simple enough for planners and engineers to use reliably.

The architecture therefore standardizes initial fluid volumes around simple primitives:

1. box,
2. sphere,
3. cylinder.

That is sufficient for the intended benchmark families and avoids turning fluid authoring into a separate modeling product.

### Fluid properties

Each fluid must support the minimum set of physically meaningful properties:

1. viscosity,
2. density,
3. surface tension,
4. render color.

This lets benchmarks distinguish water-like, syrup-like, and similar material families without demanding a full fluids database.

### Fluid objective families

The architecture currently needs two objective families.

#### Containment

Containment asks whether enough of the fluid remains inside a required region.

The contract is:

1. containment is measured as a fraction of fluid state inside the zone,
2. the benchmark specifies a threshold,
3. the benchmark may choose end-of-run or continuous evaluation,
4. failing containment is a benchmark failure, not a soft warning.

#### Flow rate

Flow-rate benchmarks ask whether fluid crosses a gate in the required quantity and direction.

The contract is:

1. the benchmark defines a gate plane,
2. the benchmark defines a target rate and tolerance,
3. the simulation produces a measured rate,
4. the episode passes only if the measured rate is within the accepted band.

For MVP and benchmarking, a particle-crossing-based approximation is acceptable. If we later want to claim real-world calibrated throughput, this section must tighten.

### Fluid-electronics coupling

Fluid contact with electronics is a hard architectural coupling, not an optional extension.

If a benchmark includes both fluids and electronics:

1. fluid contact with electrical components must fail the run,
2. that failure must be legible to the reviewer,
3. the planner must treat electronics protection as part of the design problem.

## Simulation lifecycle

### Pre-simulation gates

Before a fluid or FEM run starts, the system must validate:

1. that the benchmark selects a compatible backend,
2. that the materials needed for stress-aware simulation are defined,
3. that objective definitions are structurally valid,
4. that coupled electronics rules are not already violated by the initial setup.

### Scene build

The scene build stage must produce a simulator-ready scene that includes:

1. rigid or deformable solids,
2. fluid spawn volumes,
3. motion definitions,
4. objective markers and zones,
5. any coupled electromechanical structures required for the episode.

The precise intermediate mesh format is not part of the architecture. The stable requirement is that Genesis receives valid deformable and fluid-capable scene data.

### Main run

During the main run, the system must:

1. step the selected backend,
2. collect stress-aware signals when enabled,
3. evaluate fluid and stress objectives at the right cadence,
4. stop immediately on hard failure,
5. keep enough state to explain the outcome after the run.

### Final evaluation

At the end of the run, the system must determine:

1. success or failure,
2. failure reason when present,
3. final fluid metrics,
4. final stress summaries,
5. durable reviewer-facing artifacts.

## Failure modes and definition of done

### Failure modes

This capability introduces or elevates these failure classes:

- `FLUID_OBJECTIVE_FAILED`
- `STRESS_OBJECTIVE_EXCEEDED`
- `PART_BREAKAGE`
- `ELECTRONICS_FLUID_DAMAGE`
- `PHYSICS_INSTABILITY`
- `VALIDATION_FAILED`

The architecture does not require every internal subsystem to emit them in the same way. It does require the final episode result to classify them cleanly.

### Passing state

A fluid or stress-aware run passes only when:

1. the simulation remains stable,
2. no hard failure occurs,
3. fluid objectives pass,
4. stress objectives pass,
5. no structural breakage occurs unless the benchmark explicitly defines sacrificial behavior as part of the intended solution family, which should be rare and reviewer-visible.

### Review implication

The reviewer is not allowed to approve a design based only on a green terminal status if the run lacked the fluid/stress artifacts needed for that benchmark family.

## Artifact surface

### Required reviewer-facing artifacts

Fluid and stress-aware runs must be able to emit:

1. static preview renders,
2. simulation video or equivalent dynamic renders when appropriate,
3. fluid metrics,
4. stress summaries,
5. failure classification,
6. stress heatmaps when stress fields are available.

### Stress heatmaps

Stress heatmaps are a reviewer aid, not the primary source of truth.

The architecture requirement is:

1. the system should render them when field data exists,
2. reviewers should be able to use them to locate structural hot spots,
3. approval must still be grounded in the structured result data, not only in a color image.

### Durable versus transient data

The durable product artifacts are summaries and media, not raw solver state.

That means:

1. videos, JSON summaries, heatmaps, and stress summaries are durable,
2. raw particle state may remain worker-local and transient,
3. benchmark evaluation must not depend on long-term storage of massive raw solver dumps.

## Smoke-test mode

Genesis-heavy tasks need a fast iteration path.

The architecture therefore includes a smoke-test mode with these properties:

1. lower particle budget,
2. shorter run duration,
3. cheaper scene-building assumptions when safe,
4. result confidence marked as approximate,
5. acceptance that this mode is for iteration and triage, not final benchmark sign-off.

This mode exists to keep the engineer loop practical. It must never be confused with final evidence for a released benchmark result.

## Agent implications

### Benchmark Planner

The Benchmark Planner must be able to:

1. declare fluid tasks explicitly,
2. choose a fluid objective family,
3. decide when structural safety is part of the benchmark,
4. choose a backend consistent with those requirements.

### Engineering Planner and Engineer

The engineering side must be able to:

1. design around fluid paths and containment,
2. choose materials with structural consequences in mind,
3. inspect stress outcomes,
4. iterate against failure modes such as spilling, breakage, or unsafe stress concentration.

### Reviewer

The reviewer must be able to:

1. inspect fluid metrics,
2. inspect structural summaries,
3. understand whether a failure came from breakage, containment loss, throughput miss, or instability,
4. reject "apparently passing" solutions that relied on the wrong backend or incomplete evidence.

### Tool surface

The agent tool surface must support:

1. declaring fluids,
2. requesting stress summaries,
3. rendering stress heatmaps,
4. running simulation in both approximate and final modes.

The exact helper function names are not the architectural concern here.

## Acceptance and gates

The primary release gates for this capability live in [specs/integration-test-list.md](../integration-test-list.md).

The key groups are:

1. fluid objective evaluation and backend selection,
2. FEM material and breakage gates,
3. Genesis smoke-test behavior,
4. fluid data-retention policy,
5. end-to-end fluid benchmark workflow,
6. fluid-electronics coupling where applicable.

In practical terms, the core acceptance surface includes:

- `INT-105`
- `INT-102` through `INT-111`
- `INT-131`
- `INT-134`
- `INT-138`
- `INT-139`
- `INT-066`

## MVP boundaries and future tightening

The architecture intentionally leaves room for later tightening in these areas:

1. more calibrated flow-rate measurement,
2. stronger explicit control of compute target,
3. broader fluid seeding options,
4. more deterministic artifact generation for every stress-aware scenario,
5. fewer environment-dependent skips for Genesis-heavy integration coverage.

Those are refinement items, not reasons to keep this capability out of the architecture.
