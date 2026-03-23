# Code Review: Evaluation Failures Investigation - March 3, 2026 (Round 1)

## Executive Summary

An investigation into the 0% success rate of most agent types in the `evals/run_evals.py` suite revealed critical architectural misalignments, restrictive security policies, and execution timeouts. The system is currently "paralyzed" during the implementation phase because the roles assumed by the code do not match the allowed roles in the configuration.

## Key Findings

### 1. Agent Role & Security Policy Mismatch (Critical)

There is a significant discrepancy between the `agent_role` strings used in the Python code and the roles defined in `config/agents_config.yaml`.

- **Observed Roles in Code:**
  - `engineer_coder` (used in `controller/agent/nodes/coder.py`)
  - `electronics_engineer` (used in `controller/agent/nodes/electronics_engineer.py`)
- **Configured Roles in Policy:**
  - `engineering_mechanical_coder`
  - `engineering_electrical_coder`
- **Impact:** Since the code uses roles that don't exist in the config, they fall back to the **default read-only policy**. This prevents agents from writing `script.py`, updating `todo.md`, or creating any implementation artifacts, leading to immediate "REJECTED" reviews.

### 2. Execution & DSPy Timeouts

Multiple nodes are failing due to a hard 180-second timeout in the DSPy ReAct loops.

- **Affected Nodes:** `planner`, `electronics_planner`, and `plan_reviewer`.
- **Root Cause:** ReAct trajectories often involve 5-10 tool calls (listing files, reading templates, writing plans). If the model encounters a permission error or a validation failure, it attempts to recover, which quickly consumes the 180s window.
- **LLM Stability:** Logs show `litellm.BadRequestError: OpenAIException - Provider returned error`, suggesting that long trajectories might be hitting context window limits or triggering provider-side safety filters.

### 3. Validation Strictness vs. "Lazy" LLM Outputs

The `file_validation.py` logic is strict, while LLM outputs frequently fall short:

- **Template Pollution:** Agents often submit `plan.md` or `todo.md` that still contain placeholder text like `[part_1_name]` or `...`.
- **Missing Sections:** Frequent rejections occur because `plan.md` is missing mandatory sections: `## 4. Cost & Weight Budget` and `## 5. Risk Assessment`.
- **Numbered Lists:** The `plan_md_missing_sections` error is often triggered because the "Assembly Strategy" is not formatted as a numbered list, as required by the regex-based validator.

### 4. Workspace Initialization Conflicts

The `initialize_agent_files` function in `controller/agent/initialization.py` may be overwriting evaluation-specific data.

- **Symptom:** Logs show agents reading `benchmark_definition.yaml` files filled with template zeros (`[0,0,0]` for goal zones).
- **Cause:** `run_evals.py` seeds the workspace with specific objectives, but the controller's initialization logic (called at the start of every episode) might be overwriting these with fresh templates from `shared/assets/template_repos/`.

### 5. DSPy ReAct Stability Issues

- **KeyError: None:** Frequent crashes in `dspy/predict/react.py` where the agent fails to select a valid tool name, leading to a `KeyError`.
- **Permission Noise:** Agents spend many turns attempting to read restricted directories like `/reviews` or `/parts`, which consumes their turn limit and increases the risk of timeout.

## Recommendations for Fixes

1. **Policy Alignment:** Update `config/agents_config.yaml` to include the exact role names used in the code (`engineer_coder`, `electronics_engineer`, etc.) or update the code to use the more descriptive names in the config.
2. **Increase Timeouts:** Relax the `llm_timeout_seconds` for complex planning and coding nodes to 300s.
3. **Smart Initialization:** Modify `initialize_agent_files` to skip writing templates if a file already exists, or allow `run_evals.py` to suppress standard initialization.
4. **Improved Prompting:** Refine the `PromptManager` templates to emphasize the requirement for numbered lists in assembly strategies and the absolute prohibition of template placeholders in final submissions.
5. **Tool Permission Feedback:** Improve the `RemoteFilesystemMiddleware` to provide more constructive feedback when a permission is denied, discouraging the agent from retrying the same unauthorized action.
