# Role-Based File Access

## Context

When working in a multi-role system with permission restrictions, always check your role's allowed file patterns before attempting file operations.

## Pattern Observed

Agent attempted to read `/scripts` which was denied. The role 'engineer_planner' has allowed patterns: `['/', '.', 'config/**', 'skills/**', 'utils/**', 'objectives.yaml', 'assembly_definition.yaml', 'plan.md', 'todo.md', 'journal.md', 'renders/**']`

## Skill

Before any file operation, verify the target path matches your role's allowed patterns. Use `list_files` to explore available directories first.

## When to Apply

- Any read_file, write_file, or edit_file operation
- When receiving "Permission denied" errors
- When exploring new directories
