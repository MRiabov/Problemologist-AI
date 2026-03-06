# Research: Automatic Node Entry Validation

## Decision 1: Add a shared graph-level entry guard layer

### Decision
Use a shared pre-entry validation layer at orchestration level (graph routing) for both engineer and benchmark flows, instead of relying on node-local checks.

### Rationale
- The requirement is universal across nodes, so graph-level placement gives a single control point.
- Routing decisions (loopback/fail-fast) belong to orchestration, not inside individual node execution bodies.
- Existing node-local checks are inconsistent in scope and timing (mostly post-execution validation).

### Alternatives considered
- **Alternative A: Keep per-node local checks only**  
  Rejected because it duplicates logic, misses transitions between nodes, and cannot enforce deterministic loopback policy consistently.
- **Alternative B: Worker-side validation only**  
  Rejected because worker tools validate artifacts, not graph transition semantics or previous-node routing.

## Decision 2: Deterministic previous-node mapping tables

### Decision
Define explicit previous-node maps per graph path and use them for non-integration reroute behavior.

### Rationale
- Prevents ambiguous retry targets and hidden heuristics.
- Supports testability: same invalid entry must always produce same reroute target.
- Simplifies observability and debugging.

### Alternatives considered
- **Alternative A: Infer previous node from last executed trace**  
  Rejected because traces can be incomplete/noisy and inference can diverge under retries.
- **Alternative B: Route all failures back to planner**  
  Rejected because it over-resets workflow context and adds unnecessary loop cost.

## Decision 3: Integration mode must fail fast on entry rejection

### Decision
When `IS_INTEGRATION_TEST=true`, any node-entry validation failure transitions episode/session to `FAILED` without loopback retry.

### Rationale
- Integration suites are used as architecture contract checks; loops hide root failures and produce non-deterministic runtime.
- Fail-fast behavior matches the architecture rule explicitly added in `specs/desired_architecture.md`.

### Alternatives considered
- **Alternative A: Allow one retry before fail-fast**  
  Rejected because it still introduces loop behavior and complicates timing-sensitive tests.
- **Alternative B: Keep same behavior as production runtime**  
  Rejected because known issue is infinite/long loops during integration testing.

## Decision 4: Preserve existing post-node and handover gates

### Decision
Keep existing post-node output validation and reviewer-handover validation active; entry validation is additive.

### Rationale
- Entry validation and post-execution validation cover different failure classes.
- Removing current gates would reduce safety and regress existing architecture guarantees.

### Alternatives considered
- **Alternative A: Replace post-node checks with entry checks**  
  Rejected because output correctness can only be validated after execution.
- **Alternative B: Keep handover checks only in reviewer nodes**  
  Rejected because transition-level guardrails are required before entering reviewer logic.

## Decision 5: Subagent scope boundary

### Decision
This increment covers all first-class graph nodes. Tool-invoked helper subagents that are not represented as graph nodes are out of scope unless promoted to node transitions.

### Rationale
- Keeps feature bounded and implementable without redesigning tool runtime.
- Aligns with the user's explicit “every node, with possible subagent exception” direction.

### Alternatives considered
- **Alternative A: Include all helper subagent calls now**  
  Rejected because this requires separate contract boundaries across tool invocation internals and significantly expands scope.

## Decision 6: Observability contract through existing episode/trace surfaces

### Decision
Expose node-entry rejections via existing persisted traces/events and episode status transitions, without introducing new external endpoints.

### Rationale
- Integration tests already assert through HTTP episode polling and traces/events.
- Minimizes surface-area risk and preserves strict API stability.

### Alternatives considered
- **Alternative A: New dedicated “entry validation API” endpoint**  
  Rejected because no new user workflow requires it and it adds unnecessary API maintenance.

## Outstanding risks

- Production non-integration loopback can still repeat if upstream node never fixes root cause; mitigation is hard-fail limits and explicit reason logging.
- Single-node planner executions have no natural previous node; contract sets them to fail-closed on validation failure.
