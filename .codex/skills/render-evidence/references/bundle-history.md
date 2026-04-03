# Bundle History

## Purpose

Render output is immutable evidence. Treat each render-producing request as one bundle, not as a mutable "latest render" folder.

## What Counts as a Bundle

- An immutable directory under `renders/**`.
- A bundle-local manifest written beside the media.
- Optional sidecars such as `preview_scene.json`, `frames.jsonl`, or `objects.parquet` when the bundle needs them.
- An append-only history row in `renders/render_index.jsonl`.

## Trust Order

1. The selected bundle path.
2. The bundle-local manifest inside that bundle.
3. The append-only history row for discovery.
4. Compatibility aliases such as `renders/render_manifest.json`, if the runtime still exposes them.

If the index row, manifest, and on-disk paths disagree, fail closed. Do not guess which one is right.

## Selection Rules

- Use the latest revision only when the task is explicitly revision-scoped.
- Use historical lookup only when the task asks for replay, audit, or older evidence.
- Keep one bundle identity per judgment.
- Do not mix benchmark, engineer, and final evidence in the same reasoning chain unless the task explicitly compares them.

## Fields To Check

The bundle record should make these facts easy to recover:

- `bundle_id`
- `created_at`
- `revision`
- `scene_hash`
- `bundle_path`
- primary media paths

## Do Not

- Do not infer history from filename order.
- Do not use the root `renders/` directory as the authoritative source once bundle-local data exists.
- Do not read stale media from a previous revision when current evidence exists.
- Do not flatten render buckets back into one generic directory.
