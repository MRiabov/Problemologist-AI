# renders Acceptance Criteria

Use this reference when seeded evals include render evidence.

## Role of the File Family

The render family includes bundle-local render manifests, the append-only render index, preview scene snapshots, optional simulation sidecars, and the actual rendered media files.

## Acceptance Criteria

- Inspect the actual media, not just filenames.
- Use the latest revision's render bundle only.
- Ensure bundle-local `render_manifest.json` data matches the current seed and review gate.
- Ensure `renders/render_index.jsonl` matches the latest published bundle metadata when historical lookup is involved.
- Ensure `preview_scene.json`, `frames.jsonl`, and `objects.parquet` match the current bundle when those sidecars are present.
- Treat missing or stale render evidence as a hard failure, not a prompt to guess.
- Treat `renders/render_manifest.json` as a compatibility alias, not the historical source of truth.
