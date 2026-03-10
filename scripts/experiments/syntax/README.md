# DSPy Tool Syntax Probe

This experiment answers four narrow debugging questions:

1. Does `dspy.ReAct` in our installed DSPy version force the `next_thought` / `next_tool_name` / `next_tool_args` contract?
2. Does `dspy.ChatAdapter` silently fall back to `JSONAdapter` by default?
3. If we bypass `dspy.ReAct`, can DSPy itself still produce native tool calls through `ToolCalls`?
4. Does the provider path support native tool calls independently of DSPy ReAct?

## Run

Inspect the installed DSPy runtime only:

```bash
uv run python scripts/experiments/syntax/probe_dspy_tool_syntax.py
```

Run the live DSPy-native tool-call probe:

```bash
uv run python scripts/experiments/syntax/probe_dspy_tool_syntax.py --live-dspy-native
```

Run the direct provider-native probe:

```bash
uv run python scripts/experiments/syntax/probe_dspy_tool_syntax.py --live-openrouter
```

Run both live probes:

```bash
uv run python scripts/experiments/syntax/probe_dspy_tool_syntax.py \
  --live-dspy-native \
  --live-openrouter
```

## Interpretation

- `react_forces_next_tool_fields=true` means this DSPy ReAct version is inherently using the textual `next_tool_*` protocol.
- `chat_adapter_default.use_json_adapter_fallback=true` means malformed chat output can silently switch parsing modes unless we disable that in runtime code.
- `dspy_native_tool_probe.tool_calls_present=true` means DSPy itself is still usable if we replace `ReAct` with a custom tool loop.
- `openrouter_native_tool_probe.tool_calls_present=true` means the provider path supports native tools, so the text protocol is a DSPy runtime choice rather than an OpenRouter limitation.
