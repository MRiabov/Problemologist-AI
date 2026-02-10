# Agent Evaluations vs Observability (Feb 10, 2026)

## Scope

Question: Are current observability capabilities sufficient to enforce evaluation requirements like "used ABC in X% of cases" in `kitty-specs/desired_architecture.md` (Agent Evaluations section)?

## Short Answer

No. The current observability stack (Langfuse traces + DB `Trace` records with tool start/end and LLM output) does not capture **structured, domain-level usage signals** needed to compute metrics such as "fasteners in 70% of builds" or "motors in 20% of builds." It can show *that a tool was called* or *what the model said*, but not **what components were actually instantiated** or **what geometry/features were produced**.

## Why It Falls Short

1. **No component-level events**: There is no structured event stream for "part usage" or "feature usage" (fasteners, motors, bearings, CSG vs sketches). Tool calls and LLM outputs are too unstructured to aggregate reliably.
2. **No authoritative artifact parse**: The observability layer does not ingest the final CAD/MJCF or BOM and normalize it into a schema that can be counted.
3. **No eval-result persistence**: There is no explicit storage for evaluation outcomes (e.g., `toolkit_usage.fastener_used = true`) tied to an episode for aggregation.
4. **No deterministic measurement path**: Relying on LLM text to infer "used X" will be inconsistent and non-auditable.

## Recommendation (Persist This, Not in Desired Architecture Doc)

Implement **explicit eval signals** in observability, then compute percentages off those signals.

Minimum required additions:

1. **Structured "component usage" events** emitted by the worker (or by the build123d/MJCF export layer) for every part/category used.
   - Example: `usage.fastener = true`, `usage.bearing = false`, `usage.motor = true` per episode.
2. **Artifact-derived counts** (BOM or MJCF parse) stored as normalized fields in the observability DB.
   - This avoids depending on LLM text and ensures deterministic measurement.
3. **Eval result records** stored alongside episodes (e.g., `evaluation.toolkit_usage.fastener_used = true`) so aggregation can be done without reprocessing artifacts.
4. **Schema contract** for these fields so `schemathesis` can enforce presence/shape.

## Conclusion

Current observability is adequate for trace logging and debugging, but **not for enforcing or measuring usage-rate requirements** in Agent Evaluations. You need explicit domain events or derived artifact metrics to compute "used ABC in X% of cases" reliably.

## Decision Logged (Feb 10, 2026)

We will **not** use Langfuse as the source of truth for these evaluation metrics. The **main DB** will store structured, queryable usage/eval signals, with optional Langfuse metadata mirroring for trace-level convenience.
