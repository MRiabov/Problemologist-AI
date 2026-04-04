from __future__ import annotations

import json
from pathlib import Path

import pytest
from git import Repo

from evals.logic.skill_training import _seed_skill_overlay_state
from evals.logic.skill_promotion import promote_skill_overlay
from shared.skills import iter_skill_catalog_entries, resolve_skill_file
from worker_light.utils.git import commit_all, init_workspace_repo


def _write_skill(root: Path, name: str, description: str) -> None:
    skill_dir = root / name
    skill_dir.mkdir(parents=True, exist_ok=True)
    (skill_dir / "SKILL.md").write_text(
        "\n".join(
            [
                "---",
                f"description: {description}",
                "---",
                "",
                f"# {name}",
                "",
                description,
                "",
            ]
        ),
        encoding="utf-8",
    )


@pytest.mark.integration_p1
@pytest.mark.int_id("INT-218")
def test_int_218_skill_catalog_prefers_overlay_before_canonical(tmp_path, monkeypatch):
    canonical_root = tmp_path / "skills"
    overlay_root = tmp_path / "suggested_skills"
    _write_skill(canonical_root, "shared-skill", "canonical description")
    _write_skill(canonical_root, "canonical-only", "canonical only")
    _write_skill(overlay_root, "shared-skill", "overlay description")
    _write_skill(overlay_root, "overlay-only", "overlay only")

    monkeypatch.setenv("PROBLEMOLOGIST_SKILL_OVERLAY_ROOT", str(overlay_root))

    entries = iter_skill_catalog_entries(canonical_root=canonical_root)
    assert entries[0] == ("overlay-only", "overlay only")
    assert entries[1] == ("shared-skill", "overlay description")
    assert ("canonical-only", "canonical only") in entries

    resolved_overlay = resolve_skill_file(
        "shared-skill",
        canonical_root=canonical_root,
    )
    assert resolved_overlay == overlay_root / "shared-skill" / "SKILL.md"


@pytest.mark.integration_p1
@pytest.mark.int_id("INT-219")
def test_int_219_skill_overlay_promotion_publishes_and_records_result(tmp_path):
    workspace_dir = tmp_path
    canonical_root = workspace_dir / "skills"
    overlay_root = workspace_dir / "suggested_skills"

    _write_skill(canonical_root, "shared-skill", "canonical description")
    _write_skill(canonical_root, "canonical-only", "canonical only")
    init_workspace_repo(canonical_root)

    _write_skill(overlay_root, "shared-skill", "overlay description")
    _write_skill(overlay_root, "overlay-only", "overlay only")

    result = promote_skill_overlay(
        workspace_dir=workspace_dir,
        session_id="skill-session-001",
    )

    assert result.ok is True
    assert result.outcome == "published"
    assert result.codex_session_id == "skill-session-001"
    assert result.approved_base_commit is not None
    assert result.promotion_commit is not None
    assert result.promotion_record_path is not None
    assert (workspace_dir / result.promotion_record_path).exists()

    promoted_shared = (canonical_root / "shared-skill" / "SKILL.md").read_text(
        encoding="utf-8"
    )
    promoted_overlay = (canonical_root / "overlay-only" / "SKILL.md").read_text(
        encoding="utf-8"
    )
    assert "overlay description" in promoted_shared
    assert "overlay only" in promoted_overlay

    record = json.loads(
        (workspace_dir / result.promotion_record_path).read_text(encoding="utf-8")
    )
    assert record["outcome"] == "published"
    assert record["codex_session_id"] == "skill-session-001"
    assert record["approved_base_commit"] == result.approved_base_commit
    assert record["promotion_commit"] == result.promotion_commit


@pytest.mark.integration_p1
@pytest.mark.int_id("INT-220")
def test_int_220_skill_overlay_promotion_fails_closed_on_dirty_repo(tmp_path):
    workspace_dir = tmp_path
    canonical_root = workspace_dir / "skills"
    overlay_root = workspace_dir / "suggested_skills"

    _write_skill(canonical_root, "shared-skill", "canonical description")
    init_workspace_repo(canonical_root)
    (canonical_root / "shared-skill" / "SKILL.md").write_text(
        "\n".join(
            [
                "---",
                "description: dirty canonical description",
                "---",
                "",
                "# shared-skill",
                "",
                "dirty canonical description",
                "",
            ]
        ),
        encoding="utf-8",
    )
    _write_skill(overlay_root, "shared-skill", "overlay description")

    result = promote_skill_overlay(
        workspace_dir=workspace_dir,
        session_id="skill-session-002",
    )

    assert result.ok is False
    assert result.outcome == "conflict"
    assert result.codex_session_id == "skill-session-002"
    assert result.promotion_commit is None
    assert "uncommitted changes" in (result.reason or "")
    assert result.promotion_record_path is not None
    assert (workspace_dir / result.promotion_record_path).exists()


@pytest.mark.integration_p1
@pytest.mark.int_id("INT-221")
def test_int_221_skill_overlay_promotion_detects_base_commit_overlap(tmp_path):
    workspace_dir = tmp_path
    canonical_root = workspace_dir / "skills"
    overlay_root = workspace_dir / "suggested_skills"

    _write_skill(canonical_root, "shared-skill", "canonical description")
    _write_skill(canonical_root, "canonical-only", "canonical only")
    init_workspace_repo(canonical_root)
    base_commit = Repo(canonical_root).head.commit.hexsha

    # Advance canonical with an overlapping change after the approved base.
    _write_skill(canonical_root, "shared-skill", "canonical revised description")
    advanced_commit = commit_all(canonical_root, "Advance canonical shared-skill")
    assert advanced_commit is not None
    assert advanced_commit != base_commit

    _write_skill(overlay_root, "shared-skill", "overlay description")
    _write_skill(overlay_root, "overlay-only", "overlay only")

    result = promote_skill_overlay(
        workspace_dir=workspace_dir,
        session_id="skill-session-003",
        approved_base_commit=base_commit,
    )

    assert result.ok is False
    assert result.outcome == "conflict"
    assert result.approved_base_commit == base_commit
    assert result.merge_strategy == "base-commit-overlap"
    assert result.conflicting_skill_paths == ["shared-skill/SKILL.md"]
    assert "overlapping skill changes" in (result.reason or "")
    assert result.promotion_commit is None
    assert result.promotion_record_path is not None
    record = json.loads(
        (workspace_dir / result.promotion_record_path).read_text(encoding="utf-8")
    )
    assert record["outcome"] == "conflict"
    assert record["approved_base_commit"] == base_commit
    assert record["conflicting_skill_paths"] == ["shared-skill/SKILL.md"]


@pytest.mark.integration_p1
@pytest.mark.int_id("INT-222")
def test_int_222_skill_overlay_training_seeds_from_canonical_snapshot(tmp_path):
    workspace_dir = tmp_path
    canonical_root = workspace_dir / "skills"
    overlay_root = workspace_dir / "suggested_skills"

    _write_skill(canonical_root, "shared-skill", "canonical description")
    _write_skill(canonical_root, "canonical-only", "canonical only")
    init_workspace_repo(canonical_root)

    seeded_overlay_root, repo_root, base_commit, branch = _seed_skill_overlay_state(
        workspace_dir
    )

    assert seeded_overlay_root == overlay_root
    assert repo_root == canonical_root
    assert base_commit is not None
    assert branch is not None
    assert (overlay_root / ".git").exists()
    assert (
        overlay_root / "shared-skill" / "SKILL.md"
    ).read_text(encoding="utf-8").strip().endswith("canonical description")
    assert (overlay_root / "canonical-only" / "SKILL.md").exists()


@pytest.mark.integration_p1
@pytest.mark.int_id("INT-223")
def test_int_223_skill_overlay_promotion_applies_seeded_overlay_deletions(tmp_path):
    workspace_dir = tmp_path
    canonical_root = workspace_dir / "skills"
    overlay_root = workspace_dir / "suggested_skills"

    _write_skill(canonical_root, "shared-skill", "canonical description")
    _write_skill(canonical_root, "canonical-only", "canonical only")
    init_workspace_repo(canonical_root)
    base_commit = Repo(canonical_root).head.commit.hexsha

    _seed_skill_overlay_state(workspace_dir)
    (overlay_root / "canonical-only" / "SKILL.md").unlink()
    (overlay_root / "shared-skill" / "SKILL.md").write_text(
        "\n".join(
            [
                "---",
                "description: overlay revised description",
                "---",
                "",
                "# shared-skill",
                "",
                "overlay revised description",
                "",
            ]
        ),
        encoding="utf-8",
    )

    result = promote_skill_overlay(
        workspace_dir=workspace_dir,
        session_id="skill-session-004",
        approved_base_commit=base_commit,
    )

    assert result.ok is True
    assert result.outcome == "published"
    assert "canonical-only/SKILL.md" in result.changed_paths
    assert "shared-skill/SKILL.md" in result.changed_paths
    assert not (canonical_root / "canonical-only" / "SKILL.md").exists()
    promoted_shared = (canonical_root / "shared-skill" / "SKILL.md").read_text(
        encoding="utf-8"
    )
    assert "overlay revised description" in promoted_shared
