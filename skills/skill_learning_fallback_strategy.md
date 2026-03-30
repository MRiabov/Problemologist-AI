---
name: skill_learning_fallback_strategy
---

# Skill Learning Fallback Strategy

## Observation

When agent journals are empty or contain only initialization entries, reusable lessons cannot be extracted directly from session logs. In such cases, examine existing skill files (e.g., in `/skills/`) to understand documented patterns, avoid duplication, and infer the format and scope of valuable skills.

## Rationale

- Direct journal analysis relies on detailed struggle/solution entries.
- Empty or minimal journals require alternative sources of pattern information.
- Existing skills demonstrate what has been considered valuable and reusable.

## Application

1. If `journal.md` or other logs lack observed struggles or found solutions:
   - List directories like `/skills/` to see existing skill categories.
   - Read one or two SKILL.md files to understand structure and content.
   - Use this context to propose new skills that complement or fill gaps.
2. This approach ensures skills are added without overwriting or duplicating existing ones.

## Example Pattern

Agent encounters empty journal → checks `/skills/manufacturing-knowledge/SKILL.md` → notes pattern of "design constraints" documentation → suggests a new skill for another domain with similar constraints.
