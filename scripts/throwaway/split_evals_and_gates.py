#!/usr/bin/env python3
"""Deterministically split specs/architecture/evals-and-gates.md into 3 docs.

The split is anchor-based and preserves the source text exactly across the
resulting files. The script fails closed if the source headings move or if the
output coverage no longer matches the source line-for-line.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Iterable

DEFAULT_SOURCE = Path("specs/architecture/evals-and-gates.md")
DEFAULT_OUTPUTS = {
    Path("specs/architecture/application-acceptance-criteria.md"): [
        (0, "## Evaluations architecture"),
    ],
    Path("specs/architecture/evals-architecture.md"): [
        ("## Evaluations architecture", "## Reward config structure"),
        ("## Multi-level evaluations architecture", None),
    ],
    Path("specs/architecture/agent/reward-architecture.md"): [
        ("## Reward config structure", "## Multi-level evaluations architecture"),
    ],
}


def _count_headings(lines: Iterable[str]) -> int:
    return sum(1 for line in lines if re.match(r"^#{1,6}\s+", line))


def _find_heading(lines: list[str], heading: str) -> int:
    for idx, line in enumerate(lines):
        if line.rstrip("\n") == heading:
            return idx
    raise ValueError(f"Missing required heading: {heading}")


def _slice_ranges(lines: list[str], ranges: list[tuple[int, int]]) -> str:
    return "".join(segment for start, end in ranges for segment in lines[start:end])


def _build_partitions(lines: list[str]) -> dict[Path, list[tuple[int, int]]]:
    eval_start = _find_heading(lines, "## Evaluations architecture")
    reward_start = _find_heading(lines, "## Reward config structure")
    multi_start = _find_heading(lines, "## Multi-level evaluations architecture")

    if not (0 < eval_start < reward_start < multi_start < len(lines)):
        raise ValueError(
            "Unexpected heading order in evals-and-gates.md; the split anchors "
            "no longer match the source."
        )

    partitions: dict[Path, list[tuple[int, int]]] = {
        Path("specs/architecture/application-acceptance-criteria.md"): [
            (0, eval_start),
        ],
        Path("specs/architecture/evals-architecture.md"): [
            (eval_start, reward_start),
            (multi_start, len(lines)),
        ],
        Path("specs/architecture/agent/reward-architecture.md"): [
            (reward_start, multi_start),
        ],
    }

    covered = [
        idx
        for ranges in partitions.values()
        for start, end in ranges
        for idx in range(start, end)
    ]
    expected = list(range(len(lines)))
    if sorted(covered) != expected:
        raise ValueError(
            "The partition does not cover the source exactly. "
            "Expected a full line-for-line split."
        )

    return partitions


def _write_outputs(
    source: Path, partitions: dict[Path, list[tuple[int, int]]]
) -> dict[Path, str]:
    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    outputs: dict[Path, str] = {}
    for out_path, ranges in partitions.items():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        content = _slice_ranges(lines, ranges)
        out_path.write_text(content, encoding="utf-8")
        outputs[out_path] = content
    return outputs


def _verify_outputs(source_lines: list[str], outputs: dict[Path, str]) -> None:
    source_heading_count = _count_headings(source_lines)
    output_heading_count = sum(
        _count_headings(content.splitlines()) for content in outputs.values()
    )
    if source_heading_count != output_heading_count:
        raise ValueError(
            f"Heading count mismatch: source={source_heading_count}, "
            f"outputs={output_heading_count}"
        )

    source_line_count = len(source_lines)
    output_line_count = sum(
        len(content.splitlines(keepends=True)) for content in outputs.values()
    )
    if source_line_count != output_line_count:
        raise ValueError(
            f"Line count mismatch: source={source_line_count}, outputs={output_line_count}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Split evals-and-gates.md into acceptance, eval, and reward docs."
    )
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help="Source markdown file to split.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Verify the split anchors and counts without writing files.",
    )
    args = parser.parse_args()

    source = args.source
    if not source.exists():
        raise FileNotFoundError(f"Source file does not exist: {source}")
    if source.suffix.lower() != ".md":
        raise ValueError(f"Source file must be markdown: {source}")

    source_lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    partitions = _build_partitions(source_lines)

    if not args.check_only:
        outputs = _write_outputs(source, partitions)
    else:
        outputs = {}
        for out_path, ranges in partitions.items():
            content = _slice_ranges(source_lines, ranges)
            outputs[out_path] = content

    _verify_outputs(source_lines, outputs)

    for out_path, ranges in partitions.items():
        line_count = sum(end - start for start, end in ranges)
        heading_count = _count_headings(outputs[out_path].splitlines())
        print(f"{out_path}: {line_count} lines, {heading_count} headings")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
