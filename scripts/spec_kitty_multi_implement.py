#!/usr/bin/env python3
import argparse
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
from pathlib import Path


@dataclass
class WorkPackage:
    id: str
    title: str
    lane: str
    spec_slug: str
    spec_number: str
    file_path: Path
    last_updated: datetime | None = None


def parse_frontmatter(content: str) -> dict:
    match = re.search(r"^---\n(.*?)\n---", content, re.DOTALL)
    if not match:
        return {}

    yaml_lines = match.group(1).split("\n")
    data = {}
    for line in yaml_lines:
        if ":" in line:
            key, val = line.split(":", 1)
            data[key.strip()] = val.strip().strip('"').strip("'")
    return data


def get_last_activity_time(content: str) -> datetime | None:
    lines = content.splitlines()
    for line in reversed(lines):
        line = line.strip()
        if not line.startswith("- "):
            continue
        parts = line.split(" ", 2)
        if len(parts) < 2:
            continue
        ts_str = parts[1]
        try:
            if ts_str.endswith("Z"):
                ts_str = ts_str[:-1] + "+00:00"
            return datetime.fromisoformat(ts_str)
        except ValueError:
            continue
    return None


def get_wp_status(
    wp_file: Path, spec_slug: str, spec_number: str
) -> WorkPackage | None:
    try:
        content = wp_file.read_text()
        data = parse_frontmatter(content)
        if not data:
            return None

        return WorkPackage(
            id=data.get("work_package_id", ""),
            title=data.get("title", ""),
            lane=data.get("lane", "planned"),
            spec_slug=spec_slug,
            spec_number=spec_number,
            file_path=wp_file,
            last_updated=get_last_activity_time(content),
        )
    except Exception as e:
        print(f"Error parsing {wp_file}: {e}")
        return None


def is_spec_accepted(spec_dir: Path) -> bool:
    meta_file = spec_dir / "meta.json"
    if not meta_file.exists():
        return False
    try:
        data = json.loads(meta_file.read_text())
        return "accepted_at" in data and data["accepted_at"] is not None
    except Exception:
        return False


def is_stale(wp: WorkPackage) -> bool:
    if wp.lane != "doing" or not wp.last_updated:
        return False
    now = datetime.now(UTC)
    # Stale if older than 15 minutes
    return (now - wp.last_updated) > timedelta(minutes=15)


def main():
    parser = argparse.ArgumentParser(
        description="Run implementation for the first available work package in each spec in parallel."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands that would be executed without running them.",
    )
    args = parser.parse_args()

    base_path = Path("kitty-specs")
    if not base_path.exists():
        print("Error: kitty-specs directory not found.")
        sys.exit(1)

    print("🔍 Scanning specs for available work packages...")

    # Collect WPs by spec
    spec_wps: dict[str, list[WorkPackage]] = {}

    for spec_dir in sorted(base_path.iterdir()):
        if not spec_dir.is_dir():
            continue

        spec_name = spec_dir.name
        spec_parts = spec_name.split("-", 1)
        spec_number = spec_parts[0]
        spec_slug = spec_parts[1] if len(spec_parts) > 1 else spec_name

        tasks_dir = spec_dir / "tasks"
        if not tasks_dir.exists():
            continue

        if is_spec_accepted(spec_dir):
            continue

        current_spec_key = f"{spec_number}-{spec_slug}"
        spec_wps[current_spec_key] = []

        for wp_file in sorted(tasks_dir.glob("WP*.md")):
            wp = get_wp_status(wp_file, spec_slug, spec_number)
            if wp:
                spec_wps[current_spec_key].append(wp)

    commands_to_run = []

    print("\n📋 Candidates found:")

    for spec_key, wps in spec_wps.items():
        if not wps:
            continue

        candidate = None

        # Priority:
        # 1. 'doing' but stale
        # 2. 'planned'
        # (Ignore 'done', 'for_review', and active 'doing')

        # First check for any stale 'doing' items (restart them?)
        # Actually logic says "first work package that is planned or stale"
        # Let's look for the *first* one in the list order that meets criteria

        for wp in wps:
            if wp.lane == "planned":
                candidate = wp
                break
            elif wp.lane == "doing" and is_stale(wp):
                candidate = wp
                break
            # If we hit an active 'doing' task that isn't stale, maybe we should skip this spec?
            # The prompt says: "fetch the first work package that is planned or stale"
            # It implies we want to start working on something if nothing active is happening or if it's stuck.
            # If there is a non-stale doing task, we probably shouldn't start another one in parallel for the same spec?
            # But the user said "any first available work package... in parallel [across specs]".
            # Let's assume if there is an active (non-stale) doing task, we do NOT start a new one for this spec.
            elif wp.lane == "doing" and not is_stale(wp):
                print(f"  Skipping {spec_key}: Active task {wp.id} is in progress.")
                candidate = None
                break

        if candidate:
            print(f"  ✅ {spec_key}: Found {candidate.id} ({candidate.lane})")
            cmd = [
                "gemini",
                "--yolo",
                "--prompt",
                f"/spec-kitty.implement {candidate.id} from {candidate.spec_number}",
            ]
            commands_to_run.append((spec_key, cmd))

    if not commands_to_run:
        print("\nNo work packages found to implement.")
        return

    print(f"\n🚀 Launching {len(commands_to_run)} agents in parallel...")

    processes = []

    for spec_key, cmd in commands_to_run:
        cmd_str = " ".join(f'"{c}"' if " " in c else c for c in cmd)
        print(f"  [{spec_key}] Executing: {cmd_str}")

        if not args.dry_run:
            # We use Popen to run non-blocking
            # We might want to use start_new_session=True or similar if we want them to survive parent death?
            # But usually we want to see output.
            # Spawning 8 shells might be messy in one terminal.
            # The user asked: "spawning 8 processes/shells".
            # Let's just spawn them as background processes.
            p = subprocess.Popen(cmd)
            processes.append((spec_key, p))

    if args.dry_run:
        print("\n[Dry Run] No processes started.")
    else:
        print(
            f"\nStarted {len(processes)} processes. Press Ctrl+C to stop waiting (processes will continue running)."
        )
        try:
            # Wait for all? Or just exit?
            # User might want to keep the script running to see output?
            # Usually 'spawning shells' implies they run independently.
            # But standard Popen shares stdout/stderr.
            for spec_key, p in processes:
                p.wait()
        except KeyboardInterrupt:
            print("\nInterrupted. Sending SIGTERM to child processes...")
            for spec_key, p in processes:
                p.terminate()
            print("Done.")


if __name__ == "__main__":
    main()
