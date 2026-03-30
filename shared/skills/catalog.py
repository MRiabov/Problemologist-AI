from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SKILL_ROOT = ROOT / "skills"


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


@lru_cache(maxsize=1)
def iter_skill_catalog_entries() -> tuple[tuple[str, str], ...]:
    """Return checked-in skill names and descriptions from `SKILL.md` files."""
    if not SKILL_ROOT.exists():
        return tuple()

    entries: list[tuple[str, str]] = []
    for skill_path in sorted(SKILL_ROOT.glob("*/SKILL.md")):
        if not skill_path.is_file():
            continue
        skill_name = skill_path.parent.name
        entries.append((skill_name, _skill_description(skill_path)))
    return tuple(entries)


@lru_cache(maxsize=1)
def build_skill_catalog_lines() -> tuple[str, ...]:
    lines = ["Available skills you can read:"]
    for skill_name, description in iter_skill_catalog_entries():
        display_path = f"/skills/{skill_name}/SKILL.md"
        lines.append(f"- `{display_path}`: {description}")
    return tuple(lines)
