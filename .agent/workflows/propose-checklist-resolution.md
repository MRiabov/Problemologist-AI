---
description: Propose a resolution of checklists.
---

1. Locate the checklist file. If not specified, look in `kitty-specs/[feature-name]/checklists`.
2. For each checklist item (especially those marked as gaps or underspecified):
   - Research the codebase and relevant specifications to determine the correct behavior or requirement.
   - Propose a concrete resolution.
3. Edit the file to append your proposal under each item using this **exact** markdown format (pay attention to capitalization and colons):

   ```markdown
     - **Proposed Change:** [Your concrete proposal here]
     - **User Review:**
   ```

4. Leave the "**User Review:**" field blank for my input.
5. Ensure the indentation matches the nesting level of the checklist item.
