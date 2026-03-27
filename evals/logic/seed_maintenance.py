from __future__ import annotations

import hashlib
import json
from pathlib import Path

_PLAN_REVIEW_MANIFEST_NAMES = {
    "benchmark_plan_review_manifest.json",
    "engineering_plan_review_manifest.json",
}


def refresh_plan_review_manifest_hashes(
    artifact_dir: Path, *, fix: bool = False
) -> list[Path]:
    """Refresh deterministic artifact hashes in seeded plan-review manifests."""

    updated_manifests: list[Path] = []

    for manifest_path in sorted(artifact_dir.rglob("*.json")):
        if manifest_path.name not in _PLAN_REVIEW_MANIFEST_NAMES:
            continue

        try:
            manifest = json.loads(manifest_path.read_text())
        except Exception:
            continue

        artifact_hashes = manifest.get("artifact_hashes")
        if not isinstance(artifact_hashes, dict) or not artifact_hashes:
            continue

        updated_hashes = dict(artifact_hashes)
        changed = False
        for rel_path, expected_hash in artifact_hashes.items():
            file_path = artifact_dir / rel_path
            if not file_path.exists():
                continue

            actual_hash = hashlib.sha256(file_path.read_bytes()).hexdigest()
            if actual_hash != expected_hash:
                updated_hashes[rel_path] = actual_hash
                changed = True

        if not changed:
            continue

        updated_manifests.append(manifest_path)
        if fix:
            manifest["artifact_hashes"] = updated_hashes
            manifest_path.write_text(
                json.dumps(manifest, indent=2, ensure_ascii=False) + "\n"
            )

    return updated_manifests
