---
name: llm-output-acceptance
description: Consume and validate outputs from an upstream LLM system using structured tool calls or YAML artifacts, never substring matching, with retry-on-validation-failure.
---

# LLM Output Acceptance

Use this skill when a downstream agent, service, or workflow must accept outputs from another LLM safely and deterministically.

## Core Policy

1. Do not infer structure from freeform prose by substring matching or similar heuristics.
2. Treat freeform assistant text as advisory only.
3. Accept only:
   - typed tool-call outputs, when the result is small and atomic
   - YAML artifacts written with `write_file`, when the result is nested, multi-field, or consumed by later steps
   - YAML frontmatter embedded in a markdown document, when the YAML payload is small, logically tied to that document, and stays below 5 rows / top-level list entries
4. Document the exact accepted outputs in this skill before relying on them.

## Typed Schema Rules

Use strict typed schemas for every accepted output.

- Prefer enums over free-form strings.
- Put explicit minimum and maximum bounds on numeric values.
- Model accepted outputs with pydantic-style validators and classes rather than loose dict parsing.
- Reject extra fields by default.
- Allow extra fields only when the workflow truly needs them, and document that exception explicitly in the schema contract.

## What To Document

For each accepted output, specify:

- producer: tool name or file path
- consumer: the node or process that reads it
- required fields and allowed values
- invariants and cross-field checks
- whether unknown fields are rejected or explicitly allowed
- retry budget and escalation target

## Validation Loop

1. Parse the structured output.
2. Validate schema and invariants.
3. If validation fails, send a precise error message back to the model.
4. Keep the original task context and increment the attempt count.
5. Let the model retry without restarting the whole system.
6. If retries are exhausted, fail closed and surface the last validation error.

## Fault Ownership

Treat LLM-output errors and system errors differently.

- LLM-caused failures: schema violations, missing fields, bad enums, invalid ranges, or malformed tool output. These may be retried by sending the model a structured failure payload.
- System-caused failures: internal server errors, transport failures, exhausted LLM tokens, or other runtime faults on our side. These are not the model's fault and should fail closed without pretending the model can fix them.
- Do not retry system-owned failures as if they were output-validation errors.
- Surface system-owned failures as system status, not as model correction tasks.

## Tool Failure Envelope

When a tool or validator fails, return a structured failure payload to chat instead of a raw internal exception.

- Prefer YAML-shaped outputs for machine-readable failures.
- Keep status and error codes explicit.
- Do not leak Python tracebacks or internal exception text unless the workflow explicitly needs it.

Example:

```yaml
ok: false
tool: ValidateOutputsTool
error_code: missing_field
field: status
message: missing required field
expected: [approved, rejected]
retry_hint: set status and call again
```

## Error Messages

- Name the exact field, file, or tool payload that failed.
- Say what rule was violated.
- Say what the model should change on the next attempt.
- Avoid vague failure text like "invalid output".

## Selection Guide

- Use tool calls for a small fixed decision or a single structured action.
- Use YAML files for batches, handoffs, or anything another node will read later.
- Use YAML frontmatter only for small YAML payloads that belong to the markdown artifact itself, such as a tightly coupled story header or review note with fewer than 5 rows / top-level list entries.
- Prefer YAML when the contract is likely to evolve, because the schema can be validated independently of chat wording.
- Use CodeAct-style shell execution for complex workflows that need iterative file edits, validation, and cross-file coordination.

## Example Rule

If the model produces prose plus a structured artifact, the structured artifact is the source of truth and the prose does not change acceptance.
