#!/usr/bin/env python3
import os
import re
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class WorkPackage:
    id: str
    title: str
    lane: str
    review_status: str
    agent: str
    spec_slug: str
    spec_number: str
    file_path: Path


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


def get_wp_status(
    wp_file: Path, spec_slug: str, spec_number: str
) -> Optional[WorkPackage]:
    try:
        content = wp_file.read_text()
        data = parse_frontmatter(content)
        if not data:
            return None

        return WorkPackage(
            id=data.get("work_package_id", ""),
            title=data.get("title", ""),
            lane=data.get("lane", "planned"),
            review_status=data.get("review_status", ""),
            agent=data.get("agent", ""),
            spec_slug=spec_slug,
            spec_number=spec_number,
            file_path=wp_file,
        )
    except Exception as e:
        print(f"Error parsing {wp_file}: {e}")
        return None


def main():
    base_path = Path("kitty-specs")
    if not base_path.exists():
        print("Error: kitty-specs directory not found.")
        return

    wps: List[WorkPackage] = []

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

        for wp_file in sorted(tasks_dir.glob("WP*.md")):
            wp = get_wp_status(wp_file, spec_slug, spec_number)
            if wp:
                wps.append(wp)

    if not wps:
        print("No work packages found.")
        return

    # Group by Spec
    specs = sorted(list(set((wp.spec_number, wp.spec_slug) for wp in wps)))

    print("\n🚀 Spec-Kitty Progress Report\n" + "=" * 30)

    for spec_num, spec_slug in specs:
        spec_wps = [wp for wp in wps if wp.spec_number == spec_num]
        print(f"\n📂 [{spec_num}] {spec_slug}")

        for wp in spec_wps:
            status_char = (
                "✅"
                if wp.lane == "done"
                else "🕒"
                if wp.lane == "doing"
                else "📋"
                if wp.lane == "for_review"
                else "⚪"
            )
            lane_str = wp.lane.upper()
            if wp.lane == "for_review":
                lane_str = f"\033[93m{lane_str}\033[0m"
                if wp.review_status:
                    lane_str += f" ({wp.review_status})"
            elif wp.lane == "done":
                lane_str = f"\033[92m{lane_str}\033[0m"
            elif wp.lane == "doing":
                lane_str = f"\033[94m{lane_str}\033[0m"

            print(f"  {status_char} {wp.id}: {wp.title.ljust(40)} | Lane: {lane_str}")

        # Suggest next action for this spec
        reviewing = [wp for wp in spec_wps if wp.lane == "for_review"]
        doing = [wp for wp in spec_wps if wp.lane == "doing"]
        planned = [wp for wp in spec_wps if wp.lane == "planned"]
        done = [wp for wp in spec_wps if wp.lane == "done"]

        print("\n  👉 Next Action:", end=" ")
        if reviewing:
            wp = reviewing[0]
            print(
                f"Review {wp.id} -> \033[1mspec-kitty agent workflow review {wp.id} --spec {spec_num} --agent Antigravity\033[0m"
            )
        elif doing:
            wp = doing[0]
            print(f"Finish {wp.id} in worktree")
        elif planned:
            wp = planned[0]
            print(
                f"Implement {wp.id} -> \033[1mspec-kitty agent workflow implement {wp.id} --spec {spec_num} --agent Antigravity\033[0m"
            )
        elif len(done) == len(spec_wps) and len(spec_wps) > 0:
            print(
                f"Merge Spec -> \033[1mspec-kitty agent workflow merge {spec_num}\033[0m"
            )
        else:
            print("No immediate action identified.")


if __name__ == "__main__":
    main()
