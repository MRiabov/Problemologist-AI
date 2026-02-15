---
work_package_id: WP04
title: LangGraph Integration
lane: "doing"
dependencies: []
base_branch: main
base_commit: 35acbb99d47bea37eb74f4757408f6fe43a55180
created_at: '2026-02-15T11:25:19.422466+00:00'
subtasks: [T008, T012]
shell_pid: "193402"
---

# WP04 - LangGraph Integration

## Objective
Inject the interaction queue into the LangGraph execution loop, allowing user feedback to interrupt and redirect the agent's reasoning trajectory between tool calls.

## Context
- Research: `kitty-specs/011-interactive-steerability-and-design-feedback/research.md` (LangGraph section)
- Architecture: `specs/desired_architecture.md` (Steerability section)

## Subtasks

### T008: Implement SteeringQueue Middleware
**Purpose**: Check for queued user feedback during agent execution.
**Steps**:
1. Create `controller/graph/steerability_node.py`.
2. Implement a node or "edge-checker" that peeks into `SteerabilityService.get_queue(session_id)`.
3. If the queue is not empty:
    - Dequeue the message.
    - Transform it into a `HumanMessage` with the attached `SteerablePrompt` metadata.
    - Force the graph state to return to the `planner` node.
**Validation**:
- [ ] LangGraph trace shows an interruption and re-planning when a message is queued.

### T012: Integrate with Main Agent Loop
**Purpose**: Ensure the main agent is aware of the steerability queue.
**Steps**:
1. Update the agent graph definition in `controller/graph/agent_graph.py`.
2. Insert the `SteeringQueue` check at the exit of the `tools` execution node.
3. Ensure the agent system prompt is updated to handle `SteerablePrompt` metadata (selections, code references).
**Validation**:
- [ ] Agent acknowledges the queued message and its selections in the next turn.

## Definition of Done
- LangGraph execution loop correctly handles "mid-flight" feedback.
- Steerable metadata is successfully injected into the agent's context.
