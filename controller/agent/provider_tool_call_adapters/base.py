from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class ParsedToolCalls:
    assistant_text: str
    tool_calls: list[dict[str, Any]]
    adapter_name: str | None = None


class ProviderToolCallAdapter(Protocol):
    name: str

    def should_attempt(
        self,
        *,
        assistant_text: str,
        model_name: str | None,
    ) -> bool: ...

    def extract(
        self,
        *,
        message: object,
        assistant_text: str,
        model_name: str | None,
    ) -> ParsedToolCalls: ...
