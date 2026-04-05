from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from shared.skills import load_skills_projection_config

SOURCE = ROOT / ".agents" / "skills"
TARGETS = [ROOT / "skills", ROOT / ".codex" / "skills"]
SKILLS_CONFIG_PATH = ROOT / "config" / "skills_config.yaml"


def _iter_source_paths(root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    dirs: set[str] = set()
    projection = load_skills_projection_config(config_path=SKILLS_CONFIG_PATH)
    projected_skill_names = {
        skill_name
        for skill_name, policy in projection.items()
        if policy.get("is_for_worker_agents", False)
    }
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = Path(dirpath).relative_to(root).as_posix()
        dirnames[:] = [name for name in dirnames if name != ".git"]
        if rel_dir != ".":
            parts = Path(rel_dir).parts
            if len(parts) > 0 and parts[0] in projected_skill_names:
                dirs.add(rel_dir)
        for name in filenames:
            if name == ".git":
                continue
            rel_path = Path(dirpath, name).relative_to(root).as_posix()
            parts = Path(rel_path).parts
            if len(parts) > 1 and parts[0] not in projected_skill_names:
                continue
            files.add(rel_path)
    return files, dirs


def _remove_stale_entries(
    dst_root: Path, source_files: set[str], source_dirs: set[str]
) -> None:
    if not dst_root.exists():
        return
    for dirpath, dirnames, filenames in os.walk(dst_root, topdown=False):
        for filename in filenames:
            rel_file = Path(dirpath, filename).relative_to(dst_root).as_posix()
            if rel_file not in source_files:
                Path(dirpath, filename).unlink()
        for dirname in dirnames:
            rel_child = Path(dirpath, dirname).relative_to(dst_root).as_posix()
            child_path = Path(dirpath, dirname)
            if rel_child not in source_dirs:
                shutil.rmtree(child_path)


def _sync_tree(src_root: Path, dst_root: Path) -> list[str]:
    src_files, src_dirs = _iter_source_paths(src_root)
    dst_root.mkdir(parents=True, exist_ok=True)
    _remove_stale_entries(dst_root, src_files, src_dirs)

    for rel_dir in sorted(src_dirs):
        (dst_root / rel_dir).mkdir(parents=True, exist_ok=True)

    copied: list[str] = []
    for rel_path in sorted(src_files):
        src_path = src_root / rel_path
        dst_path = dst_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src_path, dst_path)
        copied.append(rel_path)
    return copied


def _snapshot(root: Path) -> tuple[set[str], dict[str, bytes]]:
    dirs: set[str] = set()
    snapshot: dict[str, bytes] = {}
    if not root.exists():
        return dirs, snapshot
    projection = load_skills_projection_config(config_path=SKILLS_CONFIG_PATH)
    projected_skill_names = {
        skill_name
        for skill_name, policy in projection.items()
        if policy.get("is_for_worker_agents", False)
    }
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [name for name in dirnames if name != ".git"]
        rel_dir = Path(dirpath).relative_to(root).as_posix()
        if rel_dir != ".":
            parts = Path(rel_dir).parts
            if len(parts) > 0 and parts[0] in projected_skill_names:
                dirs.add(rel_dir)
        for filename in filenames:
            if filename == ".git":
                continue
            path = Path(dirpath, filename)
            rel = path.relative_to(root).as_posix()
            parts = Path(rel).parts
            if len(parts) > 1 and parts[0] not in projected_skill_names:
                continue
            snapshot[rel] = path.read_bytes()
    return dirs, snapshot


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mirror canonical .agents/skills into runtime copies."
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Verify the mirrors match the canonical skills tree without writing.",
    )
    args = parser.parse_args()

    if not SOURCE.exists():
        raise FileNotFoundError(f"Canonical skills tree not found: {SOURCE}")

    source_snapshot = _snapshot(SOURCE)
    exit_code = 0
    for target in TARGETS:
        if args.check:
            target_snapshot = _snapshot(target)
            if target_snapshot != source_snapshot:
                print(f"{target.relative_to(ROOT)} is out of sync with .agents/skills")
                exit_code = 1
            continue

        copied = _sync_tree(SOURCE, target)
        print(f"Synced {len(copied)} files into {target.relative_to(ROOT)}")

    if args.check and exit_code == 0:
        print("Skill mirrors are in sync.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
