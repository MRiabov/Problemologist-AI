# COTS Geometry Import Runtime and Motor MVP

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration guides the implementation of the new COTS geometry import layer
described in [COTS geometry import](../../architecture/cots-geometry-import.md).

The target contract is:

1. authored scripts can instantiate a concrete COTS class from a selected
   catalog-backed `part_id`,
2. the COTS instance itself is the geometry object,
3. motors are the first supported family,
4. the returned geometry is an interface-faithful proxy, not a vendor-faithful
   replica,
5. `import_recipe` remains catalog provenance text, not runtime execution
   input,
6. benchmark-owned and engineer-owned COTS parts can share the same class path
   while remaining distinct in ownership and pricing,
7. the first release is verified through real integration tests, not unit
   stubs or template-only checks.

This is a plumbing change, not a cosmetic refactor. The repo already has
catalog lookup, a few hand-authored proxy parts, and electromechanical behavior
logic. What it does not yet have is a deterministic class-resolution contract
for COTS geometry.

## Problem Statement

The current system has four separate COTS-related concerns that are not yet
wired into one runtime class-resolution path:

1. the COTS catalog stores exact part identity and provenance,
2. the catalog indexer stores `import_recipe` text that looks like code,
3. `shared/cots/parts/motors.py` and `shared/cots/parts/electronics.py`
   contain handwritten proxy geometry,
4. authored scripts and templates do not have a first-class class-aware factory
   on the public COTS class surface.

That split creates three practical problems:

1. the authoring surface has to know too much about catalog internals,
2. the current recipe text is easy to misread as a runtime source of truth,
3. there is no explicit verification boundary for imported proxy geometry.

The architecture answer is to keep search, class resolution, and validation
separate. Search finds candidates. Class instantiation creates geometry.
Validation and pricing enforce provenance and ownership.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `utils/__init__.py` | Exports `preview`, `objectives_geometry`, `fastener_hole`, metadata classes, and submission helpers. | It only needs a change if the implementation decides to add a convenience export; the class-first factory can also live on `COTSPart` or the concrete classes. |
| `shared/cots/runtime.py` | Resolves catalog rows and carries provenance metadata. | This is the lookup layer, not the geometry layer. |
| `shared/cots/indexer.py` | Generates `import_recipe` strings and indexes COTS rows. | The generated recipe is provenance scaffolding, not runtime import behavior. |
| `shared/cots/base.py` | Provides `COTSPart` and emits usage events on initialization. | It is the right base for a class-aware catalog factory, not a generic import API. |
| `shared/cots/parts/motors.py` | Contains the current servo proxy class and motor metadata seed. | Motor resolution needs a class-aware constructor path, not a new generic helper. |
| `worker_heavy/utils/validation.py` | Instantiates `ServoMotor(size=comp.cots_part_id)` directly for some motor validation and pricing paths. | The migration must preserve or adapt existing direct callers while the class-first path lands. |
| `shared/cots/parts/electronics.py` | Contains proxy geometry for PSU, relay, switch, connector, and wire. | These families are post-MVP and should not dictate the first geometry import design. |
| `shared/workers/loader.py` | Executes authored scripts and returns their result objects. | It does not resolve catalog parts or instantiate COTS geometry. |
| `worker_heavy/utils/validation.py` | Uses COTS data for cost, weight, and provenance checks. | Validation needs to remain separate from geometry instantiation. |
| `worker_heavy/utils/file_validation.py` | Checks persisted COTS records against catalog provenance. | This is persistence validation, not runtime geometry resolution. |
| `worker_heavy/simulation/builder.py` | Derives actuator parameters from motor data when needed. | Geometry import must stay aligned with the same motor family data. |
| `shared/assets/template_repos/engineer/assembly_definition.yaml` | Documents COTS part rows in the engineer starter template. | The template does not yet explain the class-aware factory or provider contract. |
| `specs/architecture/CAD-and-other-infra.md` | Now points COTS geometry concerns at the new dedicated architecture doc. | The migration needs to implement the runtime path that the architecture doc describes. |
| `specs/architecture/electronics-and-electromechanics.md` | Treats COTS motors and electrical parts as part of electromechanical planning. | Imported geometry must fit that ownership model without becoming a new electrical system. |

## Target State

01. A class-aware factory on `COTSPart` or the concrete family class resolves a
    catalog-backed `part_id` to a concrete COTS instance.
02. The class instance itself is the geometry object. No separate `geometry()`
    wrapper is required for the MVP.
03. The factory resolves geometry through a typed class registry, not through
    dynamic execution of catalog `import_recipe` text.
04. Motor parts are the first supported provider family.
05. The motor proxy contract is interface-faithful:
    - correct envelope,
    - correct shaft axis,
    - correct mounting datum,
    - correct mount pattern or equivalent attachment footprint,
    - correct connector or cable-exit clearance when relevant.
06. Geometry fidelity is deliberately limited. The model may look similar, but
    it does not need vendor mesh detail.
07. Ownership stays external to the provider. The same class path can serve
    benchmark-owned fixtures and engineer-owned BOM parts, but the assembly
    context decides whether the part is read-only fixture geometry or a solution
    component.
08. The implementation fails closed when a part cannot be resolved, a provider
    is missing, or the geometry does not satisfy the declared interface contract.
09. Verification uses the repo's integration-test entrypoint, not a new
    unit-test-only path.
10. Declared COTS parts that appear in benchmark or solution handoffs must be
    instantiated into authored geometry; declared-but-unused COTS parts are a
    contract failure.

## Required Work

### 1. Add the class-aware factory

- Add a class-aware factory entrypoint on `COTSPart` or the concrete family
  classes, such as `from_catalog_id(part_id)`.
- Keep direct constructor calls working for the current seed classes. Existing
  `ServoMotor(size=...)` callers remain valid during the transition.
- Return the class instance itself, not a new wrapper type. The instance is the
  geometry object.
- Do not make the factory search the catalog or pick a candidate. It should
  only instantiate geometry for an already selected part.
- Keep the factory origin-only in the MVP. Placement and rotation happen in the
  authored assembly code after instantiation, using the provider's canonical
  local frame.

### 2. Introduce a typed provider registry

- Add a typed provider layer under `shared/cots/` for geometry construction.
- Model providers as explicit classes and typed adapters, not as ad hoc dicts
  or executable recipe strings.
- Each provider must answer:
  - which canonical `part_id` it serves,
  - which family or variant it implements,
  - which concrete class or factory creates the geometry,
  - which interface contract the returned geometry promises.
- Keep the registry deterministic. A given `part_id` should resolve to the same
  provider for a given catalog snapshot.
- If a provider needs alias normalization, keep that normalization in the
  provider registry or lookup layer, not in prompt text or authored scripts.

### 3. Make motors the first supported family

- Use the existing servo data and geometry seed in `shared/cots/parts/motors.py`
  as the MVP motor source of truth.
- Keep the current `ServoMotor` class as the MVP seed class. If a catalog-aware
  factory is added, keep it thin and delegate to the class constructor or a
  small adapter layer.
- Preserve direct `ServoMotor(size=...)` callers during the transition by
  routing them through the same seed class or adapter path.
- Normalize the public catalog `part_id` to the family/variant used by the
  class. For example, a catalog row such as `ServoMotor_DS3218` should resolve
  to the appropriate DS3218 motor instance, not to a guessed generic motor.
- Keep the geometry proxy lightweight:
  - body,
  - shaft stub,
  - mounting footprint or ears,
  - connector or cable exit silhouette if it affects clearance,
  - no vendor-detail surface work.
- Keep the existing motor behavior data aligned with the same family mapping so
  the geometry path and simulation path continue to agree on torque, speed, and
  current parameters.

### 4. Preserve catalog provenance without executing catalog text

- Keep `import_recipe` in the catalog for provenance and scaffolding only.
- Do not execute `import_recipe` at runtime.
- Do not teach the loader to import arbitrary catalog text.
- Keep catalog lookup as the authoritative way to resolve exact COTS identity,
  weight, unit cost, and snapshot metadata.
- If the implementation needs a human-readable class-instantiation hint for
  templates later, generate it from the same registry, then keep it commented
  out in starter material. That is future work, not the runtime contract.

### 5. Keep ownership and validation separate from geometry construction

- Keep `PartMetadata.cots_id` as the primary identity carrier on imported
  geometry.
- Keep the provider's canonical local frame explicit in the implementation.
  For motors, local `+Z` should remain the shaft axis and the mounting datum
  should be stable enough that later placement is a pure transform step.
- Preserve the existing ownership rules:
  - benchmark-owned fixtures remain read-only context,
  - engineer-owned COTS parts count toward solution cost and weight,
  - imported COTS geometry does not become a manufactured part by default.
- Do not make the class factory decide ownership.
- Keep pricing, weight, and provenance validation in the existing validation and
  file-validation paths.
- If a provider needs extra interface metadata, add it only when the current
  `PartMetadata` / `CompoundMetadata` contract cannot express the requirement.

### 6. Add integration coverage for the class-resolution path

- Add or refresh integration tests that exercise a real motor `part_id`
  through the real catalog and the real geometry builder.
- Prove that the class-aware factory can be used from authored scripts without
  any special runtime branch.
- Prove that invalid or unknown `part_id` values fail closed.
- Prove that the imported geometry preserves the expected label, metadata, and
  interface frame.
- Prove that a declared COTS part that is not instantiated into authored
  geometry fails the handoff or validation path.
- Keep the coverage aligned with the existing motor and electromechanical
  integration rows such as `INT-022`, `INT-125`, and `INT-156`, or add a
  dedicated geometry-import row if that is the cleaner test shape.

### 7. Update starter and guidance materials after the factory exists

- Once the factory is implemented, update starter templates and prompt-facing
  guidance so authors can see a commented import example.
- Keep the hint commented out. It should show authors that the component has to
  exist already, not change the runtime execution path.
- Keep the hint generation tied to the provider registry so the examples stay
  consistent with the actual supported families.

## Non-Goals

- Do not add vendor-fidelity STEP imports in this migration.
- Do not add internet scraping or vendor asset harvesting.
- Do not execute `import_recipe` text at runtime.
- Do not add a generic importer for every COTS family on day one.
- Do not move benchmark or engineer file ownership into the class factory.
- Do not add a second prompt or tool surface just for class resolution.
- Do not change motor simulation behavior beyond keeping it aligned with the
  same family data used by the new geometry path.

## Sequencing

This migration should land in a tight order so the new path stays small and
testable.

The safe order is:

1. Add the typed provider registry and the class-aware factory entrypoint.
2. Wire the motor family through the registry first.
3. Reuse the existing motor metadata and behavior data so class resolution and
   simulation stay aligned.
4. Add the integration coverage for real resolution and fail-closed behavior.
5. Update templates and prompt-facing guidance with commented class hints only
   after the factory is stable.
6. Broaden to other COTS families after motors prove out the pattern.

## Acceptance Criteria

1. A known motor `part_id` resolves through the class-aware factory without any
   extra selector fields.
2. The returned geometry is a useful proxy with the expected interface frame
   and mounting/clearance geometry.
3. `import_recipe` remains catalog provenance text only and is not executed as
   code.
4. The same class path works for benchmark-owned fixtures and engineer-owned
   COTS parts while ownership remains external to the provider.
5. Motor behavior stays aligned with the same family data used by the
   simulation path.
6. Invalid or unsupported `part_id` values fail closed.
7. The integration suite proves the class-resolution path through the repo's
   real test entrypoint.
8. Declared COTS parts that are part of the intended benchmark or solution
   design are actually instantiated into authored geometry, and declared-but-
   unused COTS parts fail the handoff or validation.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce
the new contract:

- `shared/cots/base.py` if the class-aware factory or shared base type lives
  there
- `shared/cots/providers.py` or a similarly named provider package/module for
  the typed class registry
- `shared/cots/parts/motors.py`
- `shared/cots/parts/electronics.py` if the family-specific geometry is later
  migrated into providers
- `shared/cots/runtime.py`
- `shared/cots/indexer.py`
- `worker_heavy/simulation/builder.py`
- `worker_heavy/utils/validation.py`
- `worker_heavy/utils/file_validation.py`
- `shared/assets/template_repos/engineer/assembly_definition.yaml`
- `specs/integration-test-list.md`
- `tests/integration/architecture_p1/test_manufacturing.py`
- `tests/integration/architecture_p1/test_electronics_full.py`
- a new integration test file if the motor class-resolution path deserves a dedicated
  test slice
- `specs/architecture/cots-geometry-import.md` if the implementation uncovers a
  contract gap that the architecture doc needs to capture
- `shared/agent_templates/**` or `shared/assets/template_repos/**` only if the
  commented class hint becomes part of the starter workspace after the factory
  is stable

## Open Questions

The implementation can proceed without a full answer to these, but they affect
how much refactoring the MVP needs:

1. Should the provider registry live as a single `shared/cots/providers.py`
   module, or should it be split into a small provider package from the start?
2. Should the existing `ServoMotor` class remain the compatibility wrapper, or
   should the first pass add a `from_catalog_id(...)` class-aware factory and
   leave the class as the geometry object?
3. Should the commented starter hint be generated in template materialization,
   or kept for a later prompt/template update once the class-aware factory has
   proven stable?

## Migration Checklist

Use this checklist to track the migration from architecture to runtime code and
then to verification. Do not close the migration until every unchecked item is
either completed or explicitly waived with a written rationale.

### Contract and plumbing

- [ ] Add a class-aware factory entrypoint on `COTSPart` or the concrete family
  classes.
- [ ] Introduce the typed provider registry under `shared/cots/`.
- [ ] Keep `import_recipe` as provenance text only.
- [ ] Keep benchmark-owned and engineer-owned COTS parts on the same class path
  while preserving ownership semantics.
- [ ] Keep the factory origin-only in the MVP and define the provider's
  canonical local frame as the rotation contract.
- [ ] Preserve or adapt existing direct `ServoMotor(size=...)` callers while
  the new factory lands, so validation and pricing keep working during the
  transition.
- [ ] Ensure declared COTS parts in benchmark or solution handoffs are actually
  instantiated into authored geometry, and fail the handoff when they are
  not.

### Motor MVP

- [ ] Wire the existing motor family through the class registry.
- [ ] Keep the geometry proxy interface-faithful and intentionally lightweight.
- [ ] Keep motor behavior data aligned with the same family mapping used by the
  geometry path.
- [ ] Fail closed on unsupported or unknown motor IDs.

### Verification

- [ ] Add integration tests for a known motor `part_id`.
- [ ] Add integration tests for invalid and unsupported `part_id` failures.
- [ ] Add integration tests that prove the imported geometry preserves the
  expected frame, label, and COTS metadata.
- [ ] Add integration tests that prove placement and rotation are caller-side
  concerns, not factory concerns.
- [ ] Add integration tests that prove declared-but-unused COTS parts fail the
  handoff or validation path.
- [ ] Refresh the integration catalog if a dedicated COTS geometry import row
  is the clearer test shape than extending the existing motor rows.

### Template and guidance follow-up

- [ ] Add commented class hints to the starter workspace only after the factory
  is stable.
- [ ] Keep the hint generation tied to the class registry.
- [ ] Update prompt-facing guidance only if the starter material actually
  changes.
