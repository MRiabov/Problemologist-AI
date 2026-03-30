from contextlib import suppress
from typing import Any

from .base import ParsedToolCalls
from controller.agent.runtime_models import (
    NativeProviderMessage,
    NativeToolCall,
    NativeToolCallFunction,
)


class NativeToolCallAdapter:
    name = "native"

    def should_attempt(
        self,
        *,
        assistant_text: str,
        model_name: str | None,
    ) -> bool:
        return True

    def extract(
        self,
        *,
        message: object,
        assistant_text: str,
        model_name: str | None,
    ) -> ParsedToolCalls:
        message_model = None
        with suppress(Exception):
            message_model = NativeProviderMessage.model_validate(message)
        if message_model is None:
            return ParsedToolCalls(assistant_text=assistant_text, tool_calls=[])

        tool_calls: list[dict[str, Any]] = []
        for idx, tool_call in enumerate(message_model.tool_calls):
            tool_call_model = None
            with suppress(Exception):
                tool_call_model = NativeToolCall.model_validate(tool_call)
            if tool_call_model is None:
                continue
            function = tool_call_model.function or NativeToolCallFunction()
            tool_name = (
                function.name
                or tool_call_model.name
                or tool_call_model.tool_name
                or ""
            ).strip()
            raw_arguments = (
                function.arguments
                or tool_call_model.arguments
                or tool_call_model.tool_input
                or "{}"
            )
            tool_call_id = tool_call_model.id or f"tool_call_{idx}"
            tool_call_type = tool_call_model.type or "function"
            provider_specific_fields = tool_call_model.provider_specific_fields

            payload = {
                "id": tool_call_id,
                "type": tool_call_type,
                "function": {
                    "name": tool_name,
                    "arguments": raw_arguments or "{}",
                },
            }
            if isinstance(provider_specific_fields, dict) and provider_specific_fields:
                payload["provider_specific_fields"] = provider_specific_fields

            tool_calls.append(payload)

        return ParsedToolCalls(assistant_text=assistant_text, tool_calls=tool_calls)
