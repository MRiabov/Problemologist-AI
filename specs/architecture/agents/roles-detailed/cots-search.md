# COTS Search

## Role Summary

The COTS Search subagent returns catalog-backed part candidates with exact identities.

## What It Owns

- its single structured response payload

## What It Reads

- the caller-provided query string
- `plan.md` when the caller passes file context
- `todo.md` when the caller passes file context
- `benchmark_definition.yaml` when the caller passes file context
- `benchmark_assembly_definition.yaml` when the caller passes file context
- `assembly_definition.yaml` when the caller passes file context

## Native Tool Surface

- `search_cots_catalog`
- `read_file`

## What Humans Must Tell It

- Pass one request string only.
- Ask for exact `part_id`, manufacturer, price, weight, and fit rationale.
- Tell it whether the query is for benchmark-owned fixtures or engineer-owned solution parts.
- Tell it the budget or weight ceiling when that matters.
- Do not ask it to invent proxy parts when the catalog has no exact match.

## Acceptance Checklist

- The response names exact catalog identities.
- The response is concise enough to copy into a planner handoff.
- The response does not pretend to inherit workspace plan state.

## Related Skills

- `.agents/skills/cots-parts/SKILL.md`
