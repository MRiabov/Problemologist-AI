# Tool Call Persistence Strategy

## Context

When an agent fails to produce valid tool calls for multiple iterations (e.g., iteration 3), the session can get stuck.

## Pattern Observed

System error: "Native tool loop for engineer_planner produced no tool calls on iteration 3. Max retries reached."

## Skill

Always ensure each turn produces valid tool calls with all three required fields: next_thought, next_tool_name, and next_tool_args. If stuck, review the required output format and produce a minimal valid tool call (like list_files to explore).

## When to Apply

- After receiving "no tool calls" errors
- When unsure what to do next
- As a fallback, use list_files to explore the workspace
