---
name: spec-editing-style-guide
description: Write or edit specification documents in `specs/` using the repository style contract in `specs/plan-style-traits.md`. Use whenever creating new spec files, revising existing specs, rewriting plan sections, or tightening wording/structure/acceptance criteria in spec documents; not in normal chat responses.
---

# Spec Editing Style Guide

## Core Rule

Read `specs/plan-style-traits.md` before making any edit to files in `specs/`.
Treat that file as the source of truth for tone, structure, and acceptance-criteria style.

## Workflow

1. Identify the `specs/` files being changed.
2. Open and review `specs/plan-style-traits.md` in the current workspace at the start of the edit session.
3. Check whether the same guidance is already stated elsewhere in the relevant spec set. If a point has already been repeated three times across the docs, consolidate or cross-reference it instead of restating it again. If repetition is still needed for local readability, rewrite it so the line says something meaningfully different and reflects the file's own context.
4. Apply edits to match the required style patterns from that guide.
5. Run a final pass checking that each changed section conforms to the guide before finishing.

## Guardrails

- Do not invent alternate style rules when the guide is explicit.
- Prefer direct compliance over paraphrasing the guide from memory.
- If a requested change conflicts with the guide, follow the guide and state the conflict.
- Do not repeat the same guidance more than three times across related docs. After the third occurrence, replace duplication with a short reference to the canonical section.
- If a topic appears in multiple files, each occurrence must contribute a distinct, context-relevant point rather than a near-duplicate sentence. Small contextual tweaks are acceptable only when the line remains self-contained, useful in that file, and not verbose.

## Notes

This skill currently enforces only `specs/plan-style-traits.md`.
