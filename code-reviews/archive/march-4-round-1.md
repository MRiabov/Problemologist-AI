# Code Review - March 4, Round 1

## Scope

Benchmark planner orchestration, planner tool contract, and observability plumbing.

## Findings

1. `submit_plan()` is not runtime-enforced as a hard completion gate.

- Severity: High
- Why this matters: Planner can reach `PLANNED` if artifacts validate, even if explicit planner submission never happened.
- Evidence:
  - `controller/agent/benchmark/graph.py:64`
  - `controller/agent/benchmark/graph.py:82`
  - `controller/agent/benchmark/graph.py:92`
  - `controller/agent/benchmark/tools.py:33`
- Notes: Current behavior relies on artifact state + semantic checks, not on explicit `submit_plan` success signal.

2. Benchmark planner prompt/tool contract is inconsistent.

- Severity: High
- Why this matters: Prompt advertises tools unavailable in the benchmark planner toolset, increasing risk of invalid tool calls and unstable ReAct behavior.
- Evidence:
  - `config/prompts.yaml:436`
  - `config/prompts.yaml:440`
  - `controller/agent/benchmark/tools.py:72`
- Notes: Prompt lists `execute_command`, `inspect_topology`, and pricing/COTS-related utils; toolset currently exposes only filesystem subset + `submit_plan`.

3. Node-start trace input field mismatch degrades UI visibility.

- Severity: Medium
- Why this matters: UI gets generic phase logs with empty/irrelevant input instead of useful context for benchmark planner turns.
- Evidence:
  - `controller/agent/nodes/base.py:420`
  - `controller/agent/nodes/base.py:421`
  - `controller/agent/benchmark/nodes.py:107`
- Notes: Base logger reads `inputs["task"]`; benchmark planner provides `prompt`.

4. Planner semantic validation uses brittle keyword heuristics.

- Severity: Medium
- Why this matters: Can produce false positives/negatives based on prompt wording rather than explicit schema requirements.
- Evidence:
  - `controller/agent/benchmark/graph.py:109`
  - `controller/agent/benchmark/graph.py:115`
  - `controller/agent/benchmark/graph.py:123`
- Notes: Checks like `"obstacle" in prompt` => forbid-zone required are non-standard and fragile.

5. Benchmark planner signature/input drift (`journal` passed but not declared).

- Severity: Low
- Why this matters: Indicates plumbing drift; may silently drop intended context and make behavior less predictable.
- Evidence:
  - `controller/agent/benchmark/nodes.py:109`
  - `controller/agent/benchmark/nodes.py:36`
- Notes: `BenchmarkPlannerSignature` fields do not include `journal`, but node passes it in inputs.

## Suggested Remediation Order

1. Enforce planner completion on explicit `submit_plan` success (hard gate).
2. Align benchmark planner prompt tool list with actual toolset.
3. Fix node-start logging input extraction (`task`/`prompt` fallback).
4. Replace prompt-keyword semantic checks with explicit schema/state-driven checks.
5. Resolve signature/input drift by either declaring `journal` in signature or removing that input.
