import json
import re
from typing import Any

from .base import ParsedToolCalls


def _coerce_minimax_parameter_value(raw_value: str) -> Any:
    value = raw_value.strip()
    if value.startswith('"""') and value.endswith('"""') and len(value) >= 6:
        return value[3:-3]
    if value.startswith("'''") and value.endswith("'''") and len(value) >= 6:
        return value[3:-3]
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]

    try:
        return json.loads(value)
    except Exception:
        return value


class MiniMaxToolCallAdapter:
    name = "minimax_content_tool_call"

    _wrapper_pattern = re.compile(
        r"<minimax:tool_call>(.*?)</minimax:tool_call>", re.DOTALL
    )
    _invoke_pattern = re.compile(
        r'<invoke\s+name="([^"]+)">(.*?)</invoke>',
        re.DOTALL,
    )
    _parameter_pattern = re.compile(
        r'<parameter\s+name="([^"]+)">(.*?)</parameter>',
        re.DOTALL,
    )

    def should_attempt(
        self,
        *,
        assistant_text: str,
        model_name: str | None,
    ) -> bool:
        return "<minimax:tool_call>" in assistant_text or bool(
            model_name and "minimax" in model_name.lower()
        )

    def extract(
        self,
        *,
        message: object,
        assistant_text: str,
        model_name: str | None,
    ) -> ParsedToolCalls:
        del message, model_name
        wrapper_blocks = self._wrapper_pattern.findall(assistant_text)
        if not wrapper_blocks:
            return ParsedToolCalls(assistant_text=assistant_text, tool_calls=[])

        cleaned_text = self._wrapper_pattern.sub("", assistant_text).strip()
        tool_calls: list[dict[str, Any]] = []
        for block_idx, block in enumerate(wrapper_blocks):
            for invoke_idx, invoke_match in enumerate(
                self._invoke_pattern.finditer(block)
            ):
                tool_name = invoke_match.group(1).strip()
                invoke_body = invoke_match.group(2)
                arguments: dict[str, Any] = {}
                for param_name, param_value in self._parameter_pattern.findall(
                    invoke_body
                ):
                    arguments[param_name.strip()] = _coerce_minimax_parameter_value(
                        param_value
                    )
                tool_calls.append(
                    {
                        "id": f"minimax_tool_call_{block_idx}_{invoke_idx}",
                        "type": "function",
                        "function": {
                            "name": tool_name,
                            "arguments": json.dumps(arguments, ensure_ascii=True),
                        },
                    }
                )

        return ParsedToolCalls(
            assistant_text=cleaned_text,
            tool_calls=tool_calls,
            adapter_name=self.name,
        )
