from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from datetime import UTC, datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping
from zoneinfo import ZoneInfo

from shared.enums import DatasetCurationReasonCode, GenerationKind, SeedMatchMethod
from shared.models.schemas import (
    DatasetCurationCounts,
    DatasetCurationFamilyCoverage,
    DatasetCurationManifest,
    DatasetCurationRejectedRow,
)

_CORRUPTED_CUTOFF = datetime(2026, 3, 3, 0, 0, tzinfo=ZoneInfo("Europe/Dublin"))


@dataclass(frozen=True)
class _CuratedRow:
    source_episode_id: str
    source_seed_id: str | None
    seed_dataset: str | None
    seed_match_method: SeedMatchMethod | None
    generation_kind: GenerationKind | None
    parent_seed_id: str | None
    source_family: str
    identity_hash: str | None
    created_at: datetime | None
    sort_key: tuple[str, ...]
    reasons: list[DatasetCurationReasonCode]

    @property
    def is_rejected(self) -> bool:
        return bool(self.reasons)


def _stringify(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _normalize_enum(enum_cls, value: Any) -> tuple[Any | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, enum_cls):
        return value, False
    try:
        return enum_cls(str(value).strip().lower()), False
    except Exception:
        return None, True


def _parse_datetime(value: Any) -> tuple[datetime | None, bool]:
    if value is None:
        return None, False
    if isinstance(value, datetime):
        if value.tzinfo is None:
            return value.replace(tzinfo=timezone.utc), False
        return value, False
    if not isinstance(value, str) or not value.strip():
        return None, True

    text = value.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"

    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None, True

    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc), False
    return parsed, False


def _family_from_metadata(metadata: Mapping[str, Any], *, fallback: str) -> str:
    for key in ("benchmark_family", "problem_family", "seed_dataset"):
        candidate = _stringify(metadata.get(key))
        if candidate:
            if candidate.endswith(".json"):
                return Path(candidate).stem
            return candidate
    return fallback


def _row_identity(row: Mapping[str, Any]) -> dict[str, Any]:
    metadata = row.get("metadata_vars")
    if not isinstance(metadata, Mapping):
        metadata = {}

    family_source_present = False
    for key in ("benchmark_family", "problem_family", "seed_dataset"):
        if _stringify(metadata.get(key)):
            family_source_present = True
            break

    source_episode_id = (
        _stringify(row.get("source_episode_id"))
        or _stringify(row.get("episode_id"))
        or _stringify(row.get("id"))
    )
    source_seed_id = _stringify(metadata.get("seed_id"))
    seed_dataset = _stringify(metadata.get("seed_dataset"))
    seed_match_method, seed_match_method_invalid = _normalize_enum(
        SeedMatchMethod, metadata.get("seed_match_method")
    )
    generation_kind, generation_kind_invalid = _normalize_enum(
        GenerationKind, metadata.get("generation_kind")
    )
    parent_seed_id = _stringify(metadata.get("parent_seed_id"))
    source_family = _family_from_metadata(
        metadata, fallback=_stringify(row.get("family")) or "uncategorized"
    )
    identity_hash = _stringify(row.get("identity_hash"))
    created_at, created_at_invalid = _parse_datetime(row.get("created_at"))

    reasons: list[DatasetCurationReasonCode] = []
    if source_episode_id is None:
        reasons.append(DatasetCurationReasonCode.MISSING_SOURCE_EPISODE_ID)
    if identity_hash is None:
        reasons.append(DatasetCurationReasonCode.MISSING_IDENTITY_HASH)
    if row.get("created_at") is None:
        reasons.append(DatasetCurationReasonCode.MISSING_CREATED_AT)
    elif created_at_invalid:
        reasons.append(DatasetCurationReasonCode.INVALID_CREATED_AT)
    elif created_at < _CORRUPTED_CUTOFF:
        reasons.append(DatasetCurationReasonCode.CORRUPTED_WINDOW_PRE_2026_03_03_DUBLIN)
    if metadata.get("is_integration_test") is True:
        reasons.append(DatasetCurationReasonCode.INTEGRATION_TEST_ROW)
    if generation_kind == GenerationKind.INTEGRATION_TEST:
        reasons.append(DatasetCurationReasonCode.INTEGRATION_TEST_GENERATION_KIND)
    if seed_match_method_invalid or generation_kind_invalid:
        reasons.append(DatasetCurationReasonCode.AMBIGUOUS_LINEAGE)
    if not family_source_present:
        reasons.append(DatasetCurationReasonCode.MISSING_FAMILY)

    lineage_parts = [
        source_seed_id,
        seed_dataset,
        _stringify(seed_match_method.value if seed_match_method else None),
        _stringify(generation_kind.value if generation_kind else None),
        parent_seed_id,
        identity_hash,
        source_episode_id,
    ]
    sort_key = tuple(part or "" for part in lineage_parts)

    return {
        "source_episode_id": source_episode_id,
        "source_seed_id": source_seed_id,
        "seed_dataset": seed_dataset,
        "seed_match_method": seed_match_method,
        "generation_kind": generation_kind,
        "parent_seed_id": parent_seed_id,
        "source_family": source_family,
        "identity_hash": identity_hash,
        "created_at": created_at,
        "sort_key": sort_key,
        "reasons": reasons,
    }


def build_dataset_curation_manifest(
    rows: Iterable[Mapping[str, Any]],
    *,
    family: str,
    dataset_version: str,
    agent_target: str,
    generated_at: datetime | None = None,
    dry_run: bool = False,
    bucket_counts: dict[str, int] | None = None,
) -> DatasetCurationManifest:
    """Build a strict dataset curation manifest from row-like records."""

    generated_at = generated_at or datetime.now(tz=UTC)
    snapshots = [_CuratedRow(**_row_identity(row)) for row in rows]

    eligible: list[_CuratedRow] = []
    rejected_rows: list[DatasetCurationRejectedRow] = []
    rejected_by_family: dict[str, int] = defaultdict(int)
    dropped_lineage: dict[str, list[str]] = defaultdict(list)
    seen_identity_hashes: set[str] = set()
    dedup_groups_with_drops: set[str] = set()

    def _record_rejection(
        row: _CuratedRow, reason_codes: list[DatasetCurationReasonCode]
    ) -> None:
        if not row.source_episode_id:
            raise ValueError("missing_source_episode_id")
        rejected_by_family[row.source_family] += 1
        rejected_rows.append(
            DatasetCurationRejectedRow(
                source_episode_id=row.source_episode_id,
                source_seed_id=row.source_seed_id,
                seed_dataset=row.seed_dataset,
                seed_match_method=row.seed_match_method,
                generation_kind=row.generation_kind,
                parent_seed_id=row.parent_seed_id,
                source_family=row.source_family,
                reasons=reason_codes,
            )
        )

    for row in snapshots:
        if row.reasons:
            _record_rejection(row, row.reasons)
        else:
            eligible.append(row)

    family_buckets: dict[str, list[_CuratedRow]] = defaultdict(list)
    all_family_keys: set[str] = set()
    for row in snapshots:
        all_family_keys.add(row.source_family)
        if not row.reasons:
            family_buckets[row.source_family].append(row)

    ordered_families = sorted(
        family_buckets,
        key=lambda key: (len(family_buckets[key]), key),
    )

    accepted_by_family: dict[str, list[_CuratedRow]] = {
        key: [] for key in ordered_families
    }

    for family_key in ordered_families:
        for row in sorted(family_buckets[family_key], key=lambda item: item.sort_key):
            if row.identity_hash is None:
                _record_rejection(
                    row,
                    [DatasetCurationReasonCode.MISSING_IDENTITY_HASH],
                )
                continue
            if row.identity_hash in seen_identity_hashes:
                dedup_groups_with_drops.add(row.identity_hash)
                dropped_lineage[row.identity_hash].append(row.source_episode_id)
                _record_rejection(
                    row,
                    [DatasetCurationReasonCode.DUPLICATE_LINEAGE],
                )
                continue
            accepted_by_family[family_key].append(row)
            seen_identity_hashes.add(row.identity_hash)

    accepted_rows = [
        row for family_key in ordered_families for row in accepted_by_family[family_key]
    ]

    coverage_by_family: dict[str, DatasetCurationFamilyCoverage] = {}
    for family_key in sorted(
        all_family_keys, key=lambda key: (len(family_buckets.get(key, [])), key)
    ):
        rows_for_family = accepted_by_family.get(family_key, [])
        coverage_by_family[family_key] = DatasetCurationFamilyCoverage(
            accepted=len(rows_for_family),
            rejected=rejected_by_family.get(family_key, 0),
            ordered_source_episode_ids=[
                row.source_episode_id for row in rows_for_family
            ],
            ordered_source_seed_ids=[
                row.source_seed_id for row in rows_for_family if row.source_seed_id
            ],
        )

    ordered_bucket_counts = bucket_counts or {
        "selected_rows": len(accepted_rows),
    }

    manifest = DatasetCurationManifest(
        agent_target=agent_target,
        bucket_counts=ordered_bucket_counts,
        counts=DatasetCurationCounts(
            accepted_before_pending_filter=len(eligible),
            accepted_after_pending_filter=len(accepted_rows),
            dedup_identity_groups_with_drops=len(dedup_groups_with_drops),
            rejected=len(rejected_rows),
        ),
        dataset_version=dataset_version,
        dropped_lineage={
            key: sorted(set(value)) for key, value in sorted(dropped_lineage.items())
        },
        dry_run=dry_run,
        family=family,
        generated_at=generated_at,
        rejected=rejected_rows,
        coverage_by_family=coverage_by_family,
    )
    return manifest


def load_dataset_curation_manifest(path: str | Path) -> DatasetCurationManifest:
    """Load and validate a curated dataset manifest from disk."""

    manifest_path = Path(path)
    return DatasetCurationManifest.model_validate_json(manifest_path.read_text())
