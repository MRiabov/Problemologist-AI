from __future__ import annotations

import os
from functools import lru_cache
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SKILL_ROOT = ROOT / "skills"
SKILL_OVERLAY_ENV = "PROBLEMOLOGIST_SKILL_OVERLAY_ROOT"


def _normalize_skill_root(root: Path | None) -> Path:
    if root is None:
        return SKILL_ROOT
    return root.expanduser().resolve()


def _skill_description(skill_path: Path) -> str:
    content = skill_path.read_text(encoding="utf-8")
    if not content.startswith("---\n"):
        raise ValueError(f"Skill file missing YAML frontmatter: {skill_path}")
    frontmatter_end = content.find("\n---\n", 4)
    if frontmatter_end == -1:
        raise ValueError(f"Skill file frontmatter not terminated: {skill_path}")
    frontmatter = yaml.safe_load(content[4:frontmatter_end]) or {}
    description = str(frontmatter.get("description", "")).strip()
    if not description:
        raise ValueError(f"Skill file missing description: {skill_path}")
    return description


@lru_cache(maxsize=8)
def _iter_skill_catalog_entries_from_roots(
    canonical_root: str, overlay_root: str | None
) -> tuple[tuple[str, str], ...]:
    roots: list[Path] = []
    if overlay_root:
        overlay_path = Path(overlay_root).expanduser().resolve()
        if overlay_path.exists():
            roots.append(overlay_path)
    canonical_path = Path(canonical_root).expanduser().resolve()
    if canonical_path.exists():
        roots.append(canonical_path)

    entries: list[tuple[str, str]] = []
    seen_skill_names: set[str] = set()
    for skill_root in roots:
        for skill_path in sorted(skill_root.glob("*/SKILL.md")):
            if not skill_path.is_file():
                continue
            skill_name = skill_path.parent.name
            if skill_name in seen_skill_names:
                continue
            entries.append((skill_name, _skill_description(skill_path)))
            seen_skill_names.add(skill_name)
    return tuple(entries)


def iter_skill_catalog_entries(
    *,
    overlay_root: Path | None = None,
    canonical_root: Path | None = None,
) -> tuple[tuple[str, str], ...]:
    """Return active skill names and descriptions from `SKILL.md` files.

    When an overlay root is present, it is resolved first and takes precedence
    over the checked-in canonical skill tree.
    """

    overlay_root_value = overlay_root
    if overlay_root_value is None:
        env_overlay_root = os.getenv(SKILL_OVERLAY_ENV)
        if env_overlay_root:
            overlay_root_value = Path(env_overlay_root)

    return _iter_skill_catalog_entries_from_roots(
        str(_normalize_skill_root(canonical_root)),
        str(_normalize_skill_root(overlay_root_value)) if overlay_root_value else None,
    )


def resolve_skill_file(
    skill_name: str,
    *,
    overlay_root: Path | None = None,
    canonical_root: Path | None = None,
) -> Path | None:
    """Resolve a skill file from the active overlay first, then canonical."""

    roots: list[Path] = []
    overlay_root_value = overlay_root
    if overlay_root_value is None:
        env_overlay_root = os.getenv(SKILL_OVERLAY_ENV)
        if env_overlay_root:
            overlay_root_value = Path(env_overlay_root)
    if overlay_root_value is not None:
        roots.append(_normalize_skill_root(overlay_root_value))
    roots.append(_normalize_skill_root(canonical_root))

    for skill_root in roots:
        candidate = skill_root / skill_name / "SKILL.md"
        if candidate.is_file():
            return candidate
    return None


def resolve_skill_tree_root(
    *,
    overlay_root: Path | None = None,
    canonical_root: Path | None = None,
) -> Path:
    """Return the active skill-tree root, preferring the overlay if present."""

    overlay_root_value = overlay_root
    if overlay_root_value is None:
        env_overlay_root = os.getenv(SKILL_OVERLAY_ENV)
        if env_overlay_root:
            overlay_root_value = Path(env_overlay_root)

    if overlay_root_value is not None:
        overlay_path = _normalize_skill_root(overlay_root_value)
        if overlay_path.exists():
            return overlay_path

    canonical_path = _normalize_skill_root(canonical_root)
    if canonical_path.exists():
        return canonical_path

    return canonical_path


def build_skill_catalog_lines(
    *,
    overlay_root: Path | None = None,
    canonical_root: Path | None = None,
) -> tuple[str, ...]:
    lines = ["Available skills you can read:"]
    active_overlay_root = overlay_root
    if active_overlay_root is None:
        env_overlay_root = os.getenv(SKILL_OVERLAY_ENV)
        if env_overlay_root:
            active_overlay_root = Path(env_overlay_root)
    if active_overlay_root is not None:
        lines.append(
            f"- session overlay first: `{_normalize_skill_root(active_overlay_root)}`"
        )
    for skill_name, description in iter_skill_catalog_entries(
        overlay_root=active_overlay_root,
        canonical_root=canonical_root,
    ):
        display_path = f"/skills/{skill_name}/SKILL.md"
        lines.append(f"- `{display_path}`: {description}")
    return tuple(lines)
