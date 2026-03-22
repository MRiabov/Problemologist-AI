from __future__ import annotations

from uuid import uuid4


def benchmark_prompt(body: str) -> str:
    return f"{body} [{uuid4()}]"


def feedback_comment(body: str = "Great job") -> str:
    return f"{body} [{uuid4()}]"
