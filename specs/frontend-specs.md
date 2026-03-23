# Frontend specs

This document describes the current user-facing frontend contract for the controller-first React application.
The architecture entrypoint remains `@./desired_architecture.md`.

## Scope

The frontend exposes two primary workflows:

1. Benchmark generation
2. Problem solution, also called engineering

Both workflows use the same shell: session history, chat, trace timeline, 3D/CAD or simulation viewer, and file/artifact browser.
The route decides which episode type is active and which viewer defaults are shown.

## Shared workspace contract

1. The frontend talks to the controller first for episodes, traces, assets, feedback, and run control.
2. The sidebar lists live episodes and lets the user switch between benchmark and engineer sessions.
3. The main workspace keeps the active episode in sync with the controller through polling and websocket updates.
4. The UI may show a mock-mode banner, but the displayed timeline and tool activity still come from backend episode data.

## Chat and traces

The chat timeline is a rendering of persisted backend traces for the active episode.

### Reasoning traces

1. Reasoning rows are rendered only from persisted backend trace records for the active episode, typically `LLM_END` traces with node or stage names.
2. The frontend does not synthesize reasoning rows from phase labels, tool rows, or placeholders.
3. Reasoning can appear incrementally while the run is in progress.
4. The `View reasoning` toggle only changes visibility of rendered reasoning content.
5. If a run requires reasoning and the backend returns no reasoning traces for a running or completed episode, the UI shows an explicit telemetry-missing warning instead of a silent success state.
6. When backend metadata includes `reasoning_step_index` or `reasoning_source`, the frontend preserves that metadata for ordering and labeling.

### Tool activity

1. Tool activity rows are rendered from backend tool-call traces only.
2. File and directory labels come from the trace payload, not from guessed paths.
3. Directory listing calls render the directory label as the primary name, with `/` used for root.
4. Failed tool calls are shown with a visible failure state.

### Context usage and compaction

1. The workspace shows context usage from `additional_info.context_usage.used_tokens` and `max_tokens` when the backend provides it.
2. The UI renders `conversation_length_exceeded` events with the compaction metadata provided by the backend.
3. Missing reasoning is a frontend warning only; run validity remains a backend fail-closed decision.

## Plan approval

Benchmark planning pauses for explicit user review when the backend reaches a planned state.

1. The approval control appears in the chat panel after the plan is ready.
2. The same approval action is available from the file-explorer side of the workspace.
3. The user can confirm or reject the plan and add a comment before the next stage starts.

## CAD, simulation, and files

1. The 3D viewer loads controller-served assets, usually GLB output from the backend.
2. Topology selection supports parts, faces, edges, and subassemblies through discrete selection modes.
3. The frontend treats the controller as the source of truth for assets and does not fetch worker files directly.
4. Simulation views remain time-progressive and support visible rewind/skip behavior.

## Code viewer and context capture

1. The code viewer shows the file tree, syntax highlighting, and line numbers.
2. Selecting code or topology can add structured context cards above the chat input.
3. The frontend sends selected context items to the backend as structured payloads; it does not concatenate them into a prompt locally.

## Feedback

1. Users can submit thumbs up/down feedback on terminal or completed agent output.
2. The feedback modal supports score changes, topic chips, and a free-text comment.
3. Submitted feedback is persisted through the feedback API and reflected back in episode trace metadata.

## Design baseline

1. The UI supports light and dark modes.
2. The styling target is a modern engineering tool, not a generic consumer chat layout.
3. The implementation uses Vite, React, and controller-generated TypeScript API types.
4. `three.js` powers the simulation-oriented viewer path, and `tscircuit` powers circuit rendering.

<!-- Future work: if a dedicated frontend epic file is introduced later, keep this document as the stable spec index for current contracts and move only the implementation plan details out of here. -->
