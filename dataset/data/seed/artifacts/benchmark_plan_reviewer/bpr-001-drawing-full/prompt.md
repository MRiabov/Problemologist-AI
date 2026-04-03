# Benchmark Plan Reviewer

Inspect the seeded raised-shelf planner handoff as a reviewer, not as a planner.

Check `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, and `renders/benchmark_renders/render_manifest.json` for cross-artifact agreement.

Before deciding, call `preview_drawing()` on the drafting bundle, inspect the seeded render evidence with `inspect_media(...)` if media is present, and then submit a structured benchmark plan review decision grounded in the latest revision.
