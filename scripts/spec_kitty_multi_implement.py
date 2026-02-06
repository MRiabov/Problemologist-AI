#!/usr/bin/env python3
import argparse
import json
import random
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
from pathlib import Path

RETRYABLE_ERRORS = [
    "model provider overload",
    "Rate limit reached",
    "quota exceeded",
    "Internal error occurred",
    "exhausted your capacity",
]

RED = "\033[91m"
RESET = "\033[0m"


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


def run_with_retry(spec_key: str, cmd: list[str], max_retries: int = 5):
    attempt = 0
    cmd_str = " ".join(f'"{c}"' if " " in c else c for c in cmd)

    while attempt < max_retries:
        print(f"  [{spec_key}] Attempt {attempt + 1}: Executing {cmd_str}")
        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )

        output = []
        while True:
            line = process.stdout.readline()
            if not line and process.poll() is not None:
                break
            if line:
                # Still print to console so user can see progress
                print(f"  [{spec_key}] {line.strip()}")
                output.append(line)

        returncode = process.poll()
        full_output = "".join(output)

        if returncode == 0:
            print(f"  [{spec_key}] ✅ Success!")
            return

        # Check for retryable errors
        should_retry = any(
            err.lower() in full_output.lower() for err in RETRYABLE_ERRORS
        )

        if should_retry:
            attempt += 1
            if attempt < max_retries:
                # Minimum 15s wait as requested to be safe on API side
                wait_time = 15
                print(
                    f"  [{spec_key}] ⚠️  Retryable error detected. Waiting {wait_time}s to retry... (Attempt {attempt}/{max_retries})"
                )
                time.sleep(wait_time)
                continue
            else:
                print(f"{RED}  [{spec_key}] ❌ Fatal: Max retries reached.{RESET}")
        else:
            print(
                f"{RED}  [{spec_key}] ❌ Fatal: Failed with non-retryable error (exit code {returncode}).{RESET}"
            )
            return


def main():
    parser = argparse.ArgumentParser(
        description="Run implementation for the first available work package in each spec in parallel."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print commands that would be executed without running them.",
    )
    parser.add_argument(
        "--max-retries",
        type=int,
        default=5,
        help="Maximum number of retries for overloaded model provider (default: 5).",
    )
    parser.add_argument(
        "--min-stagger",
        type=int,
        default=15,
        help="Minimum staggering delay in seconds (default: 15).",
    )
    parser.add_argument(
        "--max-stagger",
        type=int,
        default=45,
        help="Maximum staggering delay in seconds (default: 45).",
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
        if not tasks_dir.exists() or is_spec_accepted(spec_dir):
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

        for wp in wps:
            if wp.lane == "planned" or (wp.lane == "doing" and is_stale(wp)):
                candidate = wp
                break
            if wp.lane == "doing" and not is_stale(wp):
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

    print(f"\n🚀 Launching {len(commands_to_run)} agents with staggered start...")

    threads = []

    for i, (spec_key, cmd) in enumerate(commands_to_run):
        if args.dry_run:
            cmd_str = " ".join(f'"{c}"' if " " in c else c for c in cmd)
            print(f"  [{spec_key}] [Dry Run] Would execute: {cmd_str}")
            continue

        if i > 0:
            delay = random.randint(args.min_stagger, args.max_stagger)
            print(f"\n⏳ Waiting {delay}s before starting next agent...")
            time.sleep(delay)

        t = threading.Thread(
            target=run_with_retry, args=(spec_key, cmd, args.max_retries)
        )
        threads.append(t)
        t.start()

    if not args.dry_run:
        print(f"\nAll {len(threads)} agents active. Waiting for completion...")
        try:
            for t in threads:
                t.join()
        except KeyboardInterrupt:
            print(
                "\nInterrupted. Threads will continue until current command finishes."
            )


if __name__ == "__main__":
    main()
