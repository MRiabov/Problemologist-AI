#!/usr/bin/env python3
"""
Track the evolution of lines of code (LOC) for .py and .md files over repository history.

For each day with commits, finds the last commit of that day and counts
the total lines in .py and .md files at that point in time.

Usage:
    python scripts/loc_evolution.py
    python scripts/loc_evolution.py --output loc_evolution.csv
"""

import subprocess
import sys
from pathlib import Path


def get_commit_dates() -> list[str]:
    """Get sorted unique commit dates (YYYY-MM-DD) from the repository."""
    result = subprocess.run(
        ["git", "log", "--format=%ad", "--date=short"],
        capture_output=True,
        text=True,
        check=True,
    )
    dates = sorted(set(result.stdout.strip().splitlines()))
    return dates


def get_last_commit_for_date(date: str) -> str:
    """Get the hash of the last commit on a given date."""
    result = subprocess.run(
        [
            "git",
            "log",
            "--format=%H",
            "--date=short",
            f"--after={date}",
            f"--before={date}T23:59:59",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    commits = result.stdout.strip().splitlines()
    if not commits:
        # Fallback: try with --after including the date itself
        result = subprocess.run(
            [
                "git",
                "log",
                "--format=%H",
                "--date=short",
                f"--after={date}T00:00:00",
                f"--before={date}T23:59:59",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        commits = result.stdout.strip().splitlines()

    # Return the first commit (git log is newest-first)
    return commits[0] if commits else ""


def count_loc_for_commit(commit_hash: str) -> dict[str, int]:
    """Count lines of .py and .md files at a specific commit."""
    counts = {"py": 0, "md": 0}

    # Use git ls-tree to get all files at that commit
    result = subprocess.run(
        ["git", "ls-tree", "-r", "--name-only", commit_hash],
        capture_output=True,
        text=True,
        check=True,
    )

    files = result.stdout.strip().splitlines()
    py_files = [f for f in files if f.endswith(".py")]
    md_files = [f for f in files if f.endswith(".md")]

    # Count .py lines
    if py_files:
        result = subprocess.run(
            ["git", "show", f"{commit_hash}:"] + py_files,
            capture_output=True,
            text=True,
            check=False,  # May fail if files are binary
        )
        if result.returncode == 0:
            counts["py"] = result.stdout.count("\n")

    # Count .md lines
    if md_files:
        result = subprocess.run(
            ["git", "show", f"{commit_hash}:"] + md_files,
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            counts["md"] = result.stdout.count("\n")

    return counts


def count_loc_for_commit_wc(commit_hash: str) -> dict[str, int]:
    """Count lines using git show and wc -l for each file type."""
    counts = {"py": 0, "md": 0}

    for ext, key in [(".py", "py"), (".md", "md")]:
        # Get list of files with this extension
        result = subprocess.run(
            ["git", "ls-tree", "-r", "--name-only", commit_hash],
            capture_output=True,
            text=True,
            check=True,
        )
        files = [f for f in result.stdout.strip().splitlines() if f.endswith(ext)]

        if not files:
            continue

        # Count lines using git show and wc
        for file in files:
            result = subprocess.run(
                ["git", "show", f"{commit_hash}:{file}"],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode == 0:
                line_count = result.stdout.count("\n")
                counts[key] += line_count

    return counts


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Track LOC evolution for .py and .md files over repository history"
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        help="Output CSV file path (default: print to stdout)",
    )
    args = parser.parse_args()

    print("Fetching commit dates...", file=sys.stderr)
    dates = get_commit_dates()

    results = []

    for i, date in enumerate(dates, 1):
        print(f"[{i}/{len(dates)}] Processing {date}...", file=sys.stderr)

        commit_hash = get_last_commit_for_date(date)
        if not commit_hash:
            print(f"  Warning: No commits found for {date}", file=sys.stderr)
            continue

        # Get short hash for display
        short_hash = subprocess.run(
            ["git", "log", "-1", "--format=%h", commit_hash],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()

        counts = count_loc_for_commit_wc(commit_hash)

        results.append(
            {
                "date": date,
                "commit": short_hash,
                "py_loc": counts["py"],
                "md_loc": counts["md"],
            }
        )

        print(
            f"  .py: {counts['py']:>6,} lines | .md: {counts['md']:>6,} lines",
            file=sys.stderr,
        )

    # Output results
    if args.output:
        output_path = Path(args.output)
        with open(output_path, "w") as f:
            f.write("date,commit,py_loc,md_loc\n")
            for row in results:
                f.write(
                    f"{row['date']},{row['commit']},{row['py_loc']},{row['md_loc']}\n"
                )
        print(f"\nResults written to {output_path}", file=sys.stderr)
    else:
        print("\n" + "=" * 70)
        print(f"{'Date':<12} {'Commit':<10} {'Python LOC':>12} {'Markdown LOC':>12}")
        print("=" * 70)
        for row in results:
            print(
                f"{row['date']:<12} {row['commit']:<10} {row['py_loc']:>12,} {row['md_loc']:>12,}"
            )
        print("=" * 70)
        print(f"\nTotal days with commits: {len(results)}")
        if results:
            first = results[0]
            last = results[-1]
            print(
                f"First: {first['date']} - .py: {first['py_loc']:,}, .md: {first['md_loc']:,}"
            )
            print(
                f"Last:  {last['date']} - .py: {last['py_loc']:,}, .md: {last['md_loc']:,}"
            )
            print(
                f"Growth - .py: {last['py_loc'] - first['py_loc']:+,} lines, "
                f".md: {last['md_loc'] - first['md_loc']:+,} lines"
            )


if __name__ == "__main__":
    main()
