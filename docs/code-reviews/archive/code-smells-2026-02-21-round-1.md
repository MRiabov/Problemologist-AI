# Codebase Review - Feb 21, 2026 - Round 1

## Overview

This review focuses on the LangChain/DSPy integration boundary. The codebase uses a hybrid architecture where LangChain provides orchestration (LangGraph) and observability callbacks, while DSPy provides the reasoning engine (`CodeAct`). This creates unnecessary coupling, wrapper complexity, and suboptimal trace quality. A native Langfuse-DSPy integration via OpenTelemetry (`openinference-instrumentation-dspy`) would eliminate most of these issues.

Reference: <https://langfuse.com/integrations/frameworks/dspy.md>

---

## 🚨 Critical Issues

### 1. LangChain Tool Definitions Exist Only to Be Unwrapped for DSPy

**Files**: `controller/agent/tools.py`, `controller/agent/benchmark/tools.py`, `controller/agent/nodes/base.py`
**Issue**: Tools are defined with `@langchain_core.tools.tool` decorator, then immediately unwrapped back to raw functions in `BaseNode._get_tool_functions()` for DSPy compatibility.
**Evidence**:

```python
# tools.py - Defined as LangChain tools
from langchain_core.tools import tool

@tool
async def list_files(path: str = "/"):
    """List files in the workspace (filesystem)."""
    return await fs.list_files(path)
```

```python
# base.py - Immediately unwrapped
def _get_tool_functions(self, tool_factory: Callable) -> dict[str, Callable]:
    tools = tool_factory(self.ctx.fs, self.ctx.session_id)
    tool_fns = {}
    for t in tools:
        func = getattr(t, "func", getattr(t, "_run", None))
        if func:
            tool_fns[t.name] = func
    return tool_fns
```

**Impact**: High. This is pure indirection — wrapping functions in LangChain tool objects only to extract the underlying function again. DSPy `CodeAct` accepts plain Python functions with docstrings natively. The `@tool` decorator adds no value here; DSPy never sees the LangChain tool metadata.
**Recommendation**: Remove the `@tool` decorator. Define tools as plain `async def` functions with docstrings. DSPy reads function signatures and docstrings directly. Remove `_get_tool_functions()` entirely — just pass the list of functions.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. 358-Line `SafeCallbackHandler` Exists to Bridge LangChain Callbacks to Langfuse

**File**: `controller/observability/langfuse.py`
**Issue**: The entire `SafeCallbackHandler` class (lines 17–193, ~177 lines of method wrappers) exists because Langfuse tracing is routed through LangChain's callback system (`langfuse.langchain.CallbackHandler`). Every callback method (`on_llm_start`, `on_tool_start`, `on_chain_start`, etc.) is individually wrapped in try/except for resilience.
**Evidence**:

```python
class SafeCallbackHandler(BaseCallbackHandler):
    def on_llm_start(self, serialized, prompts, **kwargs):
        try:
            return self._safe_await(
                self.handler.on_llm_start(serialized, prompts, **kwargs), "on_llm_start"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_llm_start suppressed: {e}")
    # ... 12 more nearly identical methods
```

**Impact**: High. Native Langfuse-DSPy integration replaces this entire file with:

```python
from openinference.instrumentation.dspy import DSPyInstrumentor
DSPyInstrumentor().instrument()
```

This uses OpenTelemetry under the hood and captures DSPy's internal reasoning steps (CodeAct iterations, tool calls, retries) automatically — something the LangChain callback bridge cannot do because it only sees the outermost LLM call.
**Recommendation**: Replace `SafeCallbackHandler` and `get_langfuse_callback()` with the native `DSPyInstrumentor`. Keep `get_langfuse_client()` and `report_score()` as they use the Langfuse SDK directly.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. `DatabaseCallbackHandler` Unnecessarily Coupled to LangChain Callback Protocol

**File**: `controller/observability/database.py`
**Issue**: `DatabaseCallbackHandler` inherits from `langchain_core.callbacks.AsyncCallbackHandler` and implements LangChain-specific methods (`on_llm_start`, `on_tool_start`, `on_llm_end`, `on_tool_end`). It also receives a `langfuse_callback` object to extract trace IDs — coupling two unrelated concerns.
**Evidence**:

```python
class DatabaseCallbackHandler(AsyncCallbackHandler):
    def __init__(self, episode_id, langfuse_callback=None):
        ...
        self.langfuse_callback = langfuse_callback

    def _get_langfuse_id(self):
        if self.langfuse_callback and hasattr(self.langfuse_callback, "get_trace_id"):
            return self.langfuse_callback.get_trace_id()
```

**Impact**: Medium. This class serves two purposes: (1) recording traces to our internal DB, and (2) correlating them with Langfuse trace IDs. With native OpenTelemetry, the trace ID would be available from the OTel span context — no need to thread a `langfuse_callback` object through.
**Recommendation**: Decouple from `AsyncCallbackHandler`. Implement as a standalone class with explicit `record_llm_call()`, `record_tool_call()` methods called from the DSPy nodes, or hook into OpenTelemetry span events via a `SpanProcessor`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 4. `ChatOpenAI` Dependency Exists Only for Git Merge Conflict Resolution

**File**: `controller/agent/nodes/base.py`, `controller/utils/git.py`
**Issue**: `SharedNodeContext` initializes a `ChatOpenAI` instance (from `langchain-openai`) as `self.llm`. The only non-DSPy consumer of this LLM is `GitManager.sync_changes()` in `controller/utils/git.py`, which uses it to resolve skill-repo merge conflicts.
**Evidence**:

```python
# base.py
from langchain_openai import ChatOpenAI
llm = ChatOpenAI(model=settings.llm_model, temperature=0, request_timeout=600)
```

```python
# git.py
from langchain_core.messages import HumanMessage
# Uses self.llm to resolve merge conflicts
```

**Impact**: Medium. The `langchain-openai` dependency (and its transitive deps) is pulled in for a single edge-case feature. Since DSPy already wraps the same OpenAI model, this creates redundant client initialization.
**Recommendation**: Replace the `ChatOpenAI` call in `git.py` with a direct `dspy.LM` call or a plain `openai` SDK call. This allows dropping `langchain-openai` from `pyproject.toml`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. LangGraph State Uses `langchain_core.messages.BaseMessage` — This Must Stay

**Files**: `controller/agent/state.py`, all node files
**Issue** (not an issue, clarification): `AgentState.messages` is typed as `Annotated[list[BaseMessage], add_messages]`. This is core LangGraph machinery and **cannot** be removed without replacing LangGraph itself.
**Evidence**:

```python
from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages

class AgentState(BaseModel):
    messages: Annotated[list[BaseMessage], add_messages] = Field(default_factory=list)
```

**Impact**: None — this is correct usage. LangGraph requires `langchain-core` for its message and state types. The `AIMessage` imports in node files are also part of this.
**Recommendation**: No action. Keep `langchain-core` as a dependency for LangGraph. The migration only targets the observability and tool layers, not the orchestration layer.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 6. `langchain-openai` Import Used in Mock LLM for Tests

**File**: `controller/agent/mock_llm.py`
**Issue**: `MockChatOpenAI` inherits from `langchain_openai.ChatOpenAI`. If `langchain-openai` is removed, this mock needs updating.
**Evidence**:

```python
from langchain_core.messages import BaseMessage, AIMessage
from langchain_core.outputs import ChatResult, ChatGeneration
```

**Impact**: Low. Only affects test infrastructure.
**Recommendation**: If `ChatOpenAI` is removed from production code, update `MockChatOpenAI` to not inherit from it, or keep it as a dev dependency only.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Environment Variable Hacks in `get_langfuse_callback`

**File**: `controller/observability/langfuse.py`
**Issue**: The function sets `os.environ` for Langfuse keys as a workaround for callback handler initialization quirks.
**Evidence**:

```python
# We MUST use os.environ for secret_key and host as some versions of
# CallbackHandler don't accept them in the constructor.
if secret_key:
    os.environ["LANGFUSE_SECRET_KEY"] = secret_key
if host:
    os.environ["LANGFUSE_HOST"] = host
```

**Impact**: Low but concerning. Mutating `os.environ` at runtime is a side-effect that affects all threads. With the native integration, Langfuse reads from env vars once at init — no runtime mutation needed.
**Recommendation**: This goes away entirely with the native integration.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 📋 Migration Summary

### Dependencies After Migration

| Dependency | Status |
|---|---|
| `langchain-core` | **Keep** (required by LangGraph) |
| `langgraph` | **Keep** (orchestration) |
| `langchain` | **Evaluate** — may be droppable if only `langchain-core` is needed |
| `langchain-openai` | **Remove** (replace `ChatOpenAI` in `git.py`) |
| `openinference-instrumentation-dspy` | **Add** (native Langfuse-DSPy tracing) |
| `langfuse` | **Keep** (SDK for scores, client) |

### Files to Modify

1. `controller/observability/langfuse.py` — Remove `SafeCallbackHandler`, add `DSPyInstrumentor` init
2. `controller/observability/database.py` — Decouple from `AsyncCallbackHandler`
3. `controller/agent/tools.py` — Remove `@tool`, plain functions
4. `controller/agent/benchmark/tools.py` — Same
5. `controller/agent/nodes/base.py` — Remove `_get_tool_functions()`, remove `ChatOpenAI`
6. `controller/graph/agent.py` — Simplify callback wiring
7. `controller/utils/git.py` — Replace `ChatOpenAI` usage
8. `controller/agent/mock_llm.py` — Update mock if `ChatOpenAI` removed
9. `pyproject.toml` — Add `openinference-instrumentation-dspy`, drop `langchain-openai`
10. `controller/tools/fs.py` — Uses `StructuredTool`, evaluate if still needed

### Estimated Effort: ~2-3 days

### Risk: Medium

Main risk is decoupling `DatabaseCallbackHandler` from LangChain's callback lifecycle. The rest is straightforward removal of wrapper code.
