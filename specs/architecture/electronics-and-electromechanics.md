# Electronics and Electromechanical Systems

## Scope summary

- Primary focus: the architecture contract for electrical design, circuit validation, wire routing, power-gated actuation, and electromechanical review.
- Defines the stable product-facing contracts in `benchmark_definition.yaml`, `assembly_definition.yaml`, and the simulation/review artifact surface for electronics.
- Describes what the planner, electrical engineer, reviewer, backend, and frontend must support when a mechanism depends on electrical power.
- Use this file for electromechanical architecture and capability planning.
- Use [agents/overview.md](./agents/overview.md) and [agents/handover-contracts.md](./agents/handover-contracts.md) for the surrounding workflow contract.

This document describes what the system must support and how the major pieces fit together. It is not intended to mirror the current code structure.

## Objective

The platform must support engineering tasks where motion depends on electrical power, not just on geometry and controller functions.

That means the system must handle:

1. electrical power sources,
2. circuit validity,
3. wire routing in 3D space,
4. electrical failure modes that can stop a mechanism,
5. reviewer-visible artifacts for both the electrical plan and the physical route.

Without this layer, any benchmark involving powered motors, pumps, relays, or similar electromechanical behavior becomes partly fake.

## Product goals

1. Powered mechanisms require a valid electrical design, not only a mechanical controller.
2. Invalid circuits fail before expensive physics simulation runs.
3. Wires can matter physically when the benchmark requires it.
4. Electrical failures propagate into the mechanical result.
5. Reviewers can inspect both the schematic and the routed design.
6. Electrical components and wire costs count toward total solution cost and weight.

## Non-goals

This document does not require:

1. PCB design,
2. firmware or embedded programming,
3. analog signal-integrity analysis,
4. wireless or RF systems,
5. general-purpose electronics CAD as a separate product,
6. broad sensor logic beyond what later work may introduce.

The target is practical electromechanical validation for powered mechanisms, not a full EDA stack.

## Core architecture decision

### Electrical design is part of the assembly contract

We do not treat electricity as an invisible background assumption.

For explicit electromechanical tasks:

1. the benchmark declares electrical requirements,
2. the engineer provides an electrical design,
3. simulation validates that design before or during motion,
4. reviewers inspect the electrical evidence alongside the mechanical result.

This rule applies to engineer-owned electromechanical solutions. Benchmark-owned electronics and powered fixtures follow a different contract defined below, and that contract is intentionally weaker because it is validation context rather than the final solution.

### Benchmark fixture electronics are read-only environment behavior

Benchmarks may include motors, bearings, relays, sensors, power supplies, and other electronics as benchmark-owned fixtures when they make the task more realistic or more interesting. These are validation fixtures, not engineer-owned deliverables.

The architecture rule is:

1. benchmark-owned electronics are read-only environment fixtures, not part of the engineer-owned electrical design,
2. strict COTS identification still applies to benchmark-owned catalog parts,
3. benchmark-owned electronics do not contribute to engineer manufacturability validation, engineer price, or engineer weight totals,
4. benchmark-owned powered fixtures may be implicitly powered in MVP,
5. benchmark-owned powered fixtures do not require wiring realism unless the benchmark explicitly makes wiring part of the task,
6. benchmark-owned motion still requires reviewer-visible dynamic evidence and a stable handoff contract,
7. benchmark-owned fixtures are validation setup, not engineer-owned solution parts, so they are not required to be manufacturable as final deliverables.

### Planning is split, implementation is unified

We split electrical planning from mechanical planning, but we do not split electrical coding into a second implementation owner after the mechanical coder.

For explicit electromechanical tasks:

1. `Engineering Planner` owns the mechanical plan,
2. `Electronics Planner` augments that plan with electrical requirements, component choices, and wiring intent,
3. `Engineering Plan Reviewer` approves the combined handoff,
4. `Engineering Coder` implements the whole approved solution in one revision,
5. `Electronics Reviewer` performs specialist electrical review on the unified implementation,
6. `Engineering Execution Reviewer` performs final post-success execution review.

We choose this because electromechanical implementation details are often coupled:

1. wire routing can require geometry changes,
2. connector access can require bracket or frame changes,
3. PSU placement can require packaging and center-of-mass changes,
4. late serialized electrical-only coding creates avoidable planner drift and rework loops.

The architecture therefore keeps specialist electrical reasoning in planning and review, but keeps implementation ownership unified.

### Circuit validity comes before physics

The electrical design must pass a circuit-validation gate before the physics run is allowed to proceed.

This matters because:

1. short circuits should not become "weird simulation failures",
2. impossible power budgets should not waste expensive simulation time,
3. the engineer should receive a crisp electrical failure instead of a vague mechanical miss.

### Electricity gates actuation instead of replacing it

The system already has controller functions for motors and moving parts.

This layer does not replace controller logic. It adds a power gate:

1. controller logic still says how a motor tries to move,
2. the electrical layer decides whether it can move,
3. the effective motion is the controller command scaled by electrical power availability.

## Stable artifact contracts

### `benchmark_definition.yaml`

Benchmark-level electrical requirements live in `benchmark_definition.yaml` under `electronics_requirements`.

The stable contract is:

```yaml
electronics_requirements:
  power_supply_available:
    type: mains_ac_rectified
    voltage_dc: 24.0
    max_current_a: 10.0
  wiring_constraints:
    max_total_wire_length_mm: 2000
    restricted_zones:
      - name: keepout
        min: [0, 0, 0]
        max: [50, 50, 50]
  circuit_validation_required: true
```

This section declares what the task expects and what limits the engineer must respect. It is benchmark-owned, not an implementation artifact.

When the benchmark itself contains electronics or powered moving fixtures, benchmark-owned fixture declarations also live in `benchmark_definition.yaml`.

The stable fixture-facing contract is:

```yaml
benchmark_fixture_electronics:
  fixtures:
    - fixture_id: turntable_motor
      cots_part_id: nema17_stepper_42x40
      component_type: MOTOR
      fixture_behavior:
        motion_kind: motorized_revolute
        axis: [0, 0, 1]
        operating_range_deg: [0, 360]
        control_mode: constant_speed
        speed_deg_per_s: 180
        availability: always_on
      electrical_contract:
        implicit_power: true
        wiring_mode: omitted
      allows_engineer_interaction: true
```

This contract is benchmark-owned environment metadata. It tells engineering what fixture behavior exists, not how to implement it.

Engineer interaction with benchmark-owned electronics is allowed only when the specific benchmark-owned component declares `allows_engineer_interaction: true`. The engineer may then use the intended interaction surface, but may not rewrite benchmark wiring or mutate benchmark-owned electrical design.

### `assembly_definition.yaml`

The actual electrical design lives inside `assembly_definition.yaml` under `electronics`.

The stable structure is:

```yaml
electronics:
  power_supply:
    type: mains_ac_rectified
    voltage_dc: 24.0
    max_current_a: 10.0

  components:
    - component_id: motor_1
      type: MOTOR
      assembly_part_ref: motor_1
      rated_voltage: 24.0
      stall_current_a: 2.0
    - component_id: relay_1
      type: RELAY
      cots_part_id: srd_05vdc_sl_c

  wiring:
    - wire_id: w_pos
      from: {component: supply, terminal: v+}
      to: {component: motor_1, terminal: +}
      gauge_awg: 18
      length_mm: 120.0
      routed_in_3d: false
    - wire_id: w_path
      from: {component: supply, terminal: v+}
      to: {component: motor_1, terminal: +}
      gauge_awg: 18
      length_mm: 160.0
      routed_in_3d: true
      waypoints:
        - [-40, 0, 10]
        - [0, 0, 15]
        - [20, 0, 15]
```

### Design intent

This split between `benchmark_definition.yaml` and `assembly_definition.yaml` is intentional:

1. the benchmark declares requirements and constraints,
2. the planner handoff declares the concrete electrical solution intent and constraints,
3. the reviewer can judge whether the solution actually satisfies the benchmark instead of whether it simply contains electrical-looking data.

Implementation ownership rule:

1. `assembly_definition.yaml.electronics` is planner-owned after plan approval,
2. `Engineering Coder` implements that approved electrical design in `solution_script.py` and helper modules,
3. implementation changes that require planner-level electrical redesign must route through plan refusal / replanning, not through a separate late electrical implementer node.

## Circuit model contract

### Circuit engine

We use PySpice with ngspice as the architecture-level circuit-validation toolchain.

The point of that choice is not analog perfection. The point is deterministic reasoning about:

1. connectivity,
2. node voltages,
3. current draw,
4. invalid topologies.

### What the circuit gate must catch

Before a powered simulation runs, the electrical layer must be able to reject:

1. short circuits,
2. open circuits,
3. floating nodes,
4. current draw above the available supply,
5. wires that are undersized for the requested current when they are used as purely logical conductors.

### Supported component families

The stable component families for this architecture are:

1. motors,
2. switches,
3. relays,
4. connectors,
5. power supplies.

This is enough for the current benchmark families. If later work needs richer component models, they should extend this set deliberately instead of inventing component types ad hoc in prompts.

### Power budget

The electrical plan must be judged against available supply capacity.

The architecture requires:

1. total expected draw must be comparable against supply capacity,
2. under-provisioned designs must fail or be rejected before approval,
3. reviewers must be able to see the margin between required and available current,
4. the planner should choose credible supply headroom rather than infinite-budget electrical designs.

Whether the system uses a quick static budget, SPICE-derived budget, or both is secondary. The stable contract is that the power budget is explicit and reviewer-visible.

### Transient analysis

Transient circuit behavior is useful for some tasks, but it is not the primary gate for this WP.

The architecture requirement is:

1. transient analysis should be available when a task needs switching behavior over time,
2. the mandatory pre-simulation gate remains the simpler validity and power-budget check,
3. we do not require oscilloscope-grade electrical analysis to validate a powered mechanical benchmark.

## Power-gated actuation

### Basic rule

A motorized mechanism only works when both of these are true:

1. the controller commands motion,
2. the electrical design delivers power.

This makes electricity a real part of the engineering problem instead of decorative metadata.

Benchmark-owned powered fixtures are the exception:

1. benchmark-owned controller behavior may be treated as already powered when the benchmark contract declares `implicit_power: true`,
2. this exception is allowed only for read-only benchmark fixtures,
3. engineer-authored powered mechanisms still require explicit electrical design and power validation.

This exception only applies to read-only benchmark fixtures and does not relax the normal physics or manufacturability requirements for engineer-authored mechanisms.

### Runtime coupling

The simulation loop must support:

1. building an electrical power map,
2. scaling controller commands by that power map,
3. updating that map when the electrical state changes,
4. propagating electrical failures into the mechanical outcome.

### Backward compatibility

Legacy episodes without an explicit `electronics` section may continue to behave as implicitly powered.

That is a compatibility rule, not a design rule for new electromechanical tasks. New electronics benchmarks should state the electrical design explicitly.

Benchmark-owned powered fixtures follow the same philosophy: implicit power is allowed only when the benchmark declares it explicitly as fixture behavior. We do not want silent powered motors that exist only in code and not in the benchmark contract.

## Physical wire contract

### Logical versus physical wires

The system must support two modes:

1. logical electrical connectivity only,
2. physical routed wires that can interact with geometry and motion.

This distinction matters because not every powered task needs wire physics, but some tasks absolutely do.

### Routed wires

When a wire is physical, the architecture requires:

1. waypoint-based routing in 3D,
2. explicit gauge selection,
3. explicit length,
4. a flag that says the route participates in physical simulation.

### Clearance

Physical wire routes must be validated against the surrounding geometry.

The architecture rule is:

1. wires must not pass through solid parts away from attachment regions,
2. moving mechanisms need extra routing care around sweep volumes,
3. the system should enforce a baseline static clearance rule before simulation starts.

`2 mm` is the current baseline clearance expectation for the route validator and is a stable enough default to document.

### Slack and motion

If a wire reaches a moving part, the route must include realistic service slack.

The reviewer should reject routes that are:

1. taut where motion is expected,
2. forced through a joint or sweep envelope,
3. built from unstable short zig-zag segments that are clearly not intended as a real harness path.

### Wire tear

Physical wires are allowed to fail mechanically.

The architecture requires:

1. tension monitoring during simulation,
2. gauge-dependent tensile limits,
3. a failure state when a wire tears,
4. loss of power to the affected path after the tear.

This is important because it makes wire routing a real engineering tradeoff instead of an afterthought.

## Fluid-electronics coupling

Electronics must be treated as vulnerable to fluid exposure.

If a benchmark includes both fluids and electronics:

1. fluid contact with electrical components is a hard failure,
2. the planner must consider sealing, shielding, or separation as part of the solution,
3. the reviewer must treat fluid exposure as a first-class electrical failure, not a minor warning.

## Agent workflow

### Dedicated roles

Electronics are part of the active engineering workflow.

The architecture includes:

1. an electronics planner,
2. an electronics reviewer.

Electrical planning, 3D harness routing constraints, and circuit review remain distinct concerns from pure geometry authoring, but implementation ownership stays with the unified `Engineering Coder` described earlier in this document and in the core agent workflow specs.

### Responsibility split

The expected flow is:

1. the mechanical side defines the structure and moving parts,
2. the electrical side defines power, components, and wiring,
3. the reviewer checks both the electrical validity and the mechanical implications,
4. conflicts can route back from electronics to mechanics when the wire path is infeasible.

### Handover expectation

The handover between mechanical and electrical work must be concrete enough that the electronics side is not inventing the assembly from scratch.

The electrical side needs:

1. motor and actuator locations,
2. assembly geometry relevant to routing,
3. benchmark electrical constraints,
4. the planner's intended powered behavior.

The mechanical side, in turn, must receive back:

1. a completed electrical section,
2. wire-routing implications,
3. any required geometry changes caused by clearance or harness constraints.

## Pricing, COTS, and frontend

### Pricing

Electromechanical solutions must include electrical BOM impact.

That means total cost and weight must include:

1. electrical COTS components,
2. wire cost by length,
3. wire weight by gauge and length.

The system is not allowed to treat electricity as free.

### COTS

The COTS search surface must support electrical parts with the same seriousness as mechanical ones.

At minimum, electrical search must cover:

1. power supplies,
2. relays,
3. connectors,
4. wires,
5. motors where they are electrical BOM items rather than only mechanical actuators.

### Frontend and review surface

Electronics must be visible in review tools, not only in backend logs.

The frontend/review surface should support:

1. a schematic view,
2. a 3D wire-route view,
3. the normal model-viewer context,
4. traceable linkage between electrical artifacts and the reviewed episode.

## Failure modes and definition of done

### Failure modes

The important failure classes for this architecture are:

- short circuit,
- open circuit,
- overcurrent,
- wire torn,
- electronics fluid damage,
- validation failed.

The precise enum granularity may evolve, but the product behavior must keep those outcomes distinct enough for the engineer and reviewer to act on them.

### Passing state

An electromechanical design passes only when:

1. the electrical plan is valid,
2. the power budget is credible,
3. the routed wires are acceptable for the task,
4. the mechanical system behaves correctly under power,
5. no electrical failure occurs during the run,
6. the benchmark objective is satisfied.

### Review implication

The reviewer must reject solutions that:

1. only work because electricity was implicitly assumed instead of specified,
2. hide routing conflicts behind logical-only wires when the task required physical wiring,
3. pass motion objectives while violating electrical constraints,
4. omit the evidence needed to inspect the electrical design.

## Artifact surface

### Electrical artifacts

The durable artifact surface for electromechanical work must include:

1. the electrical section in `assembly_definition.yaml`,
2. circuit-validation results,
3. pricing and weight that include electrical items,
4. schematic and route visualization when available,
5. simulation results that identify electrical failure modes.

### Review artifacts

Reviewers should not need to reconstruct the electrical design from scattered traces.

The system should present:

1. the schematic,
2. the routed wires when physical wiring matters,
3. the outcome of the circuit-validation gate,
4. any failure classification from the simulation run.

## Acceptance and gates

The main release gates for this capability live in [specs/integration-test-list.md](../integration-test-list.md).

The key groups are:

1. circuit validation and schema gates,
2. wire-tear and routing-conflict behavior,
3. full electromechanical workflow,
4. power-budget checks,
5. electrical COTS search and costing,
6. transient analysis when required,
7. fluid-electronics coupling.

In practical terms, the core acceptance surface includes:

- `INT-120`
- `INT-121`
- `INT-122`
- `INT-123`
- `INT-124`
- `INT-126`
- `INT-128`
- `INT-132`
- `INT-136`
- `INT-137`
- `INT-140`
- `INT-141`
- `INT-066`

## MVP boundaries and future tightening

The architecture intentionally leaves room for later tightening in these areas:

1. more granular external electrical failure taxonomy,
2. stricter fail-closed behavior in every electrical subpath,
3. richer dynamic circuit analysis when a benchmark truly needs it,
4. more exact cable physics across every backend/runtime mode,
5. stronger explicit handling of legacy implicit-power episodes.

Those are refinement tasks. They do not change the central architecture decision that electronics are already part of the active product surface.
