# Journalling Agent

## Role Summary

The Journalling Agent compresses the run context into `journal.md`.

## What It Owns

- `journal.md`

## What It Reads

- workspace files that explain the current run
- validation and simulation outputs
- review artifacts
- render evidence
- prompt snapshots and task history when needed

## Native Tool Surface

- `list_files`
- `read_file`
- `grep`
- `write_file`
- `edit_file`
- `execute_command`

## What Humans Must Tell It

- Preserve decisions, blockers, open questions, and next steps.
- Keep the journal concise enough to remain useful for later debugging.
- Do not use it to redesign the solution or rewrite other task artifacts.
- Preserve traceability to the current episode or revision when possible.

## Acceptance Checklist

- `journal.md` reflects the actual run.
- The important failures and decisions are preserved.
- The journal does not overwrite the source artifacts it summarizes.

## Related Skills

- `.agents/skills/agent-skill.md`
- `.agents/skills/render-evidence/SKILL.md` when journal entries summarize visual evidence
