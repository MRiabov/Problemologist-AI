from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "skills"
TARGETS = [ROOT / ".agents" / "skills", ROOT / ".codex" / "skills"]


def _iter_source_paths(root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    dirs: set[str] = set()
    for dirpath, dirnames, filenames in os.walk(root):
        rel_dir = Path(dirpath).relative_to(root).as_posix()
        if rel_dir != ".":
            dirs.add(rel_dir)
        dirnames[:] = [name for name in dirnames if name != ".git"]
        for name in filenames:
            if name == ".git":
                continue
            rel_path = Path(dirpath, name).relative_to(root).as_posix()
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
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [name for name in dirnames if name != ".git"]
        rel_dir = Path(dirpath).relative_to(root).as_posix()
        if rel_dir != ".":
            dirs.add(rel_dir)
        for filename in filenames:
            if filename == ".git":
                continue
            path = Path(dirpath, filename)
            rel = path.relative_to(root).as_posix()
            snapshot[rel] = path.read_bytes()
    return dirs, snapshot


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mirror canonical skills into runtime copies."
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
                print(f"{target.relative_to(ROOT)} is out of sync with skills/")
                exit_code = 1
            continue

        copied = _sync_tree(SOURCE, target)
        print(f"Synced {len(copied)} files into {target.relative_to(ROOT)}")

    if args.check and exit_code == 0:
        print("Skills mirrors are in sync.")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
