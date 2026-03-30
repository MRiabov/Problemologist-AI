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

# Eval Display Environment Probe

This companion experiment records the environment behavior that affected recent
Codex eval runs:

1. Does `build_codex_env()` preserve the host `DISPLAY` and `XAUTHORITY` for local eval workspaces?
2. Does the spawned agent process inherit that `DISPLAY` and keep the host
   `XAUTHORITY`?
3. Does `worker_heavy.utils.vtk_display.ensure_headless_vtk_display()` stay on
   the ambient display path and fail closed if it is unusable?

## Run

Inspect the launcher and runtime display behavior:

```bash
uv run python scripts/experiments/syntax/probe_eval_display_env.py
```

## Interpretation

- `codex_env.DISPLAY=:0` means the eval launcher is preserving the host display
  instead of synthesizing a fallback display.
- `codex_env.XAUTHORITY=/run/user/...` means the Codex child env preserved the
  host Xauthority cookie, which is required for `DISPLAY=:0` to be usable on
  this machine.
- `current_env.DISPLAY` shows what the shell session itself is advertising.
- `ambient_vtk_display` populated with a display number means the worker render
  bootstrap accepted the ambient display.
- `ambient_vtk_display_error` populated instead means the render bootstrap
  failed closed before rendering because the ambient display was unusable.
