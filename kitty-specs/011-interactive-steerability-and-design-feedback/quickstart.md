# Quickstart: Interactive Steerability

## Using @-Mentions
Type `@` in the chat to see an autocomplete list of parts and subassemblies from the current Bill of Materials.
Example: `@bracket_main: Why is this part so heavy?`

## Targeted Code Edits
Reference specific line ranges using `@filename:L1-L2`. The UI will highlight the code snippet, and the agent will focus its edits on that range.
Example: `@model.py:45-50: increase the fillet radius to 5mm.`

## Geometric Pointing
1. Toggle the selection mode in the CAD Viewer (Face / Part / Subassembly).
2. Click the feature you want to discuss. It will highlight.
3. Type your instructions and send. The agent will receive a snapshot of that feature from the best isometric angle.

## Graceful Feedback
If the agent is currently working, you can still send messages. They will be marked as "Queued" and automatically delivered as the next turn input as soon as the current tool call finishes.
