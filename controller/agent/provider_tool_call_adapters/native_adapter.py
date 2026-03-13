from typing import Any

from .base import ParsedToolCalls


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
        tool_calls: list[dict[str, Any]] = []
        for idx, tool_call in enumerate(getattr(message, "tool_calls", []) or []):
            if isinstance(tool_call, dict):
                function = tool_call.get("function") or {}
                tool_name = (
                    function.get("name")
                    or tool_call.get("name")
                    or tool_call.get("tool_name")
                    or ""
                ).strip()
                raw_arguments = (
                    function.get("arguments")
                    or tool_call.get("arguments")
                    or tool_call.get("tool_input")
                    or "{}"
                )
                tool_call_id = tool_call.get("id", f"tool_call_{idx}")
                tool_call_type = tool_call.get("type", "function")
                provider_specific_fields = tool_call.get("provider_specific_fields")
            else:
                function = getattr(tool_call, "function", None)
                tool_name = (
                    getattr(function, "name", None)
                    or getattr(tool_call, "name", None)
                    or getattr(tool_call, "tool_name", None)
                    or ""
                ).strip()
                raw_arguments = (
                    getattr(function, "arguments", None)
                    or getattr(tool_call, "arguments", None)
                    or getattr(tool_call, "tool_input", None)
                    or "{}"
                )
                tool_call_id = getattr(tool_call, "id", f"tool_call_{idx}")
                tool_call_type = getattr(tool_call, "type", "function")
                provider_specific_fields = getattr(
                    tool_call, "provider_specific_fields", None
                )

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
