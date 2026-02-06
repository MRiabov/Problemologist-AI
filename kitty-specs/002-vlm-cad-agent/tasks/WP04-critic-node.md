---
work_package_id: WP04
title: Critic Node & Simulation
lane: "for_review"
dependencies: []
base_branch: 002-vlm-cad-agent-WP03
base_commit: 7fce1f3fe1eae6376a18110b7bb6b38e83240a06
created_at: '2026-02-06T14:37:16.312987+00:00'
subtasks: [T015, T016, T017, T018, T019]
shell_pid: "721153"
agent: "gemini"
---

## Objective

Implement the Critic node, which evaluates the Engineer's output. It checks simulation results and workbench reports (cost/manufacturability) to decide whether to approve or reject the solution.

## Context

The Critic acts as a quality gate. It does not write code; it reads logs and reports. If the simulation fails or cost is too high, it rejects the solution and provides feedback.

## Subtasks

### T015: Implement `src/agent/nodes/critic.py`

Create the Critic node function.

- **File**: `src/agent/nodes/critic.py`
- **Function**: `critic_node(state: AgentState) -> dict`
- **Logic**:
  - Read generic outputs from the previous step (Engineer).
  - Check for specific artifacts (e.g., `simulation_report.json`, `workbench_report.md` - or just parse the scratchpad if logged there).
  - Use LLM to evaluate success.

### T016: Implement logic to parse Simulation outcomes

- **Implementation**:
  - Expect the Engineer to have run a `simulate` command.
  - The Critic should look for the output of this command.
  - Identify Pass/Fail and any error messages.

### T017: Implement logic to parse Workbench outcomes

- **Implementation**:
  - Expect `validate_and_price` output.
  - Check if Cost < Budget (if budget is in state/requirements).
  - Check if Manufacturable = True.

### T018: Implement decision logic

- **Logic**:
  - **Approve**: If Sim=Pass AND Mfg=True AND Cost=OK.
    - Update state (e.g., `state["status"] = "approved"`).
    - Or route to 'End'.
  - **Reject**:
    - Provide constructive feedback.
    - Update `state["messages"]` with feedback.
    - Route back to Engineer (for small fixes) or Architect (for total rethink).

### T019: Test Critic node

- **File**: `tests/agent/test_critic.py`
- **Test**:
  - Mock successful simulation -> Expect Approval.
  - Mock failed simulation -> Expect Rejection.

## Definition of Done

- Critic correctly gates progress based on simulation/workbench results.

## Activity Log

- 2026-02-06T14:37:16Z – Gemini – shell_pid=654855 – lane=doing – Assigned agent via workflow command
- 2026-02-06T14:39:52Z – Gemini – shell_pid=654855 – lane=for_review – Ready for review: Critic node implemented with artifact evaluation and routing.
- 2026-02-06T15:44:39Z – gemini – shell_pid=721153 – lane=doing – Started review via workflow command
- 2026-02-06T16:06:03Z – gemini – shell_pid=721153 – lane=for_review – Ready for review: Implemented Critic Node with verification logic
- 2026-02-06T16:14:54Z – gemini – shell_pid=721153 – lane=for_review – Ready for review: Implemented Critic Node with verification logic
