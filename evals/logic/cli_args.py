from __future__ import annotations

import contextlib
import json
import re
from collections.abc import Iterable

_DELIMITER_PATTERN = re.compile(r"\s*(?:,|\bor\b|\|)\s*", flags=re.IGNORECASE)


def parse_cli_list_values(raw_values: Iterable[str]) -> list[str]:
    parsed: list[str] = []
    for raw in raw_values:
        token = raw.strip()
        if not token:
            continue

        if token.startswith("[") and token.endswith("]"):
            with contextlib.suppress(json.JSONDecodeError):
                loaded = json.loads(token)
                if isinstance(loaded, list):
                    for item in loaded:
                        item_text = str(item).strip().strip("'\"")
                        if item_text:
                            parsed.append(item_text)
                    continue

        token = token.strip("[]")
        parts = _DELIMITER_PATTERN.split(token)
        parsed.extend(
            part.strip().strip("[]").strip("'\"") for part in parts if part.strip()
        )
    return parsed


def parse_cli_int_set(
    raw_values: Iterable[str],
    *,
    minimum: int | None = None,
    maximum: int | None = None,
    label: str = "value",
) -> set[int]:
    parsed: set[int] = set()
    for value in parse_cli_list_values(raw_values):
        try:
            parsed_value = int(value)
        except ValueError as exc:
            raise SystemExit(f"Invalid {label} '{value}'.") from exc

        if minimum is not None and parsed_value < minimum:
            raise SystemExit(
                f"Invalid {label} '{parsed_value}'. Expected an integer between "
                f"{minimum} and {maximum if maximum is not None else 'unbounded'}."
            )
        if maximum is not None and parsed_value > maximum:
            raise SystemExit(
                f"Invalid {label} '{parsed_value}'. Expected an integer between "
                f"{minimum if minimum is not None else 'unbounded'} and {maximum}."
            )

        parsed.add(parsed_value)
    return parsed
