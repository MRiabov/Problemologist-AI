# Frontend Specification

This document defines the secondary React dashboard for Problemologist-AI.
The dashboard is the operator surface for inspecting benchmark and engineering episodes, steering active runs, reviewing generated artifacts, and recording feedback.

The frontend is controller-first. It reads episode state, traces, assets, and feedback state from the controller APIs and controller-backed polling and event updates. WebSockets are deferred to a later phase. It does not own simulation, CAD execution, worker filesystem access, or trace synthesis.

This spec aligns with the system architecture index in `./desired_architecture.md`, the agent trace and filesystem contracts in `./architecture/agents/artifacts-and-filesystem.md`, the observability contract in `./architecture/observability.md`, and the living UI component inventory in `../docs/component-inventory.md`.

## 1. Product Scope

The dashboard has one job: make agent episodes inspectable and steerable without exposing the worker or backend implementation details.

It must support:

- engineer episodes for solution work,
- benchmark episodes for benchmark generation and confirmation,
- trace inspection with reasoning and tool activity,
- artifact review for code, YAML, JSON, simulations, and circuit data,
- 3D and simulation inspection,
- explicit feedback on agent output.

It must not become a general-purpose CAD editor or an alternate backend console.

## 2. Primary Routes

| Route | Surface | Purpose |
| --- | --- | --- |
| `/` | Engineer Workspace | Inspect and steer engineering episodes, review CAD and simulation outputs, and monitor cost/weight context. |
| `/benchmark` | Benchmark Pipeline | Author benchmark prompts, inspect benchmark artifacts, and explicitly confirm plans before execution. |
| `/settings` | Settings | Switch theme and reasoning visibility preferences. |

The sidebar keeps navigation, episode history, and creation controls stable across routes.

## 3. Implementation Anchors

| Surface | Component anchors | Responsibility |
| --- | --- | --- |
| App shell | `AppLayout`, `Sidebar` | Fixed sidebar, route outlet, global feedback modal, mock-mode banner. |
| Workspace shell | `UnifiedGeneratorView` | Shared split-panel layout for engineer and benchmark workspaces. |
| Chat and traces | `ChatWindow`, `TraceList`, `ChatInput`, `ContextCards` | Episode timeline, composer, context steering, and trace feedback affordances. |
| Artifacts | `ArtifactView`, `HighlightedContent`, `ObjectivesForm` | File tree, syntax-highlighted viewer, plan/objective controls. |
| Visualization | `DesignViewer`, `ModelViewer`, `ModelBrowser`, `SimulationResults` | 3D model, simulation, heatmap, and electronics viewing. |
| Feedback | `FeedbackSystem` | Trace-scoped thumbs up/down with topic and comment capture. |
| Preferences | `Settings`, `ThemeContext`, `UISettingsContext` | Theme selection and reasoning visibility. |

## 4. Shared Shell

`AppLayout` defines the persistent app shell.

- The sidebar is fixed-width and remains visible across routes.
- The main content area renders the active route.
- A global feedback modal is mounted at the shell level so feedback can be opened from trace rows or sidebar shortcuts.
- A mock-mode banner appears when the backend indicates mocked reasoning is active.

The shell uses explicit, route-stable state instead of hidden client-side assumptions.

## 5. Workspace Composition

`EngineerWorkspace` and `BenchmarkGeneration` share the same workbench layout through `UnifiedGeneratorView`.

The default shape is:

- a left chat column,
- a right work area,
- a top viewport,
- a bottom artifact inspector.

The two right-side regions are vertically resizable, and the left/right split is horizontally resizable.
Layout sizes persist in local storage so the workspace opens with the same proportions the user last chose.

The engineer workspace overlays cost and weight metadata when those values are available.
The benchmark pipeline replaces that overlay with benchmark-centric simulation context.

The shared shell is intentionally dense and technical. It is optimized for long inspection sessions rather than for casual browsing.

## 6. Sidebar And Episode Navigation

The sidebar is the main routing and history surface.

- It links to the engineer workspace, benchmark pipeline, and settings page.
- It shows the episode history for the current work area.
- It filters sessions by prompt text or episode id.
- It lets the user create a new engineer run or a new benchmark run from the current route.
- It exposes quick thumbs up/down actions on episode history cards.

The episode list is controller-backed.
Selecting an episode loads the full episode payload, restores it into the workspace, and keeps route selection consistent with the episode type.

The sidebar status markers are explicit:

- running episodes show a pulsing indicator,
- completed episodes show success,
- failed episodes show failure.

## 7. Chat And Trace Timeline

`ChatWindow` is the center of the operational experience.
It renders the selected episode task, the trace timeline, the composer, the context cards, and the plan-confirmation controls.

### Trace Rendering Rules

Trace rendering must follow the backend trace contract. The frontend must not invent rows, reorder the backend timeline, or synthesize fake reasoning.

- reasoning spans render only from persisted backend reasoning traces,
- tool activity renders from persisted backend tool traces,
- error banners render from persisted backend error traces,
- timeline compaction and length-limit events render as explicit system events,
- user messages render from persisted user log traces.

Reasoning visibility is a presentation toggle only. It does not alter backend capture behavior.
If a reasoning-required run has no reasoning traces, the UI shows a visible telemetry-missing warning rather than pretending the run is healthy.

### Trace Affordances

- Reasoning spans are collapsible.
- Tool rows are clickable and can jump to the referenced artifact when one exists.
- The latest assistant output exposes trace feedback controls.
- The sidebar also exposes quick feedback shortcuts for the latest episode trace.

### Active Run State

While an episode is running:

- the composer switches from send to stop,
- a visible thinking indicator appears,
- trace rows continue to append as updates arrive,
- the user can interrupt the episode through the stop control.

### Failure State

When an episode fails:

- the workspace shows a terminal failure card,
- validation logs become visible,
- the active run remains inspectable so the user can diagnose the failure from traces and artifacts.

## 8. Composer And Context Steering

`ChatInput` is the primary steering surface.

It must support:

- free-form prompts,
- `@` mentions for files and CAD nodes,
- line-range steering references such as `@script.py:L10-L20`,
- keyboard submission on Enter,
- explicit stop behavior while the run is active,
- an expandable planning/objectives panel.

The composer builds structured metadata for the backend rather than concatenating prompt content in the browser.

The metadata payload includes:

- CAD mentions,
- code references,
- geometric selections,
- the current selected-context list.

Autocomplete is context-aware:

- files from the active episode are suggested,
- topology nodes from the model browser are suggested,
- valid mentions are highlighted,
- invalid mentions are marked as invalid before submission.

### Context Cards

`ContextCards` render the currently selected steering context.

- Code, CAD, and circuit context are displayed as removable cards.
- Each card can be removed individually.
- A clear-all action resets the active context set.

The context surface is a visible commitment to what the user is sending to the agent.
The frontend must not hide selected context inside opaque request payloads.

### Planning Controls

The composer exposes planning-objective fields for benchmark work.

- max cost,
- max weight,
- target quantity.

These fields are only visible when the planning panel is open.
If the selected episode already exists, the user can update objectives without creating a new run.

## 9. Plan Confirmation And Execution Gate

The frontend surfaces the backend planning state as an explicit decision point.

For benchmark work:

- once the plan is ready, the UI shows confirm and request-change controls,
- confirmation is available from both the chat card and the file explorer,
- the comment field is optional but available,
- approval is a user action, not a silent state transition.

For engineering work:

- the same decision surface is used to proceed from planning into execution,
- the control state remains visible only when the episode is in a planned state,
- the user can request changes rather than immediately confirming.

The frontend never decides whether a plan is acceptable. It only exposes the controller-owned decision surface.

## 10. Artifact Inspection

`ArtifactView` is the inspection surface for plan and episode assets.

It must provide:

- a file tree,
- syntax-highlighted content,
- line numbers for code,
- clickable line ranges for context injection,
- explicit empty states when no artifact is selected.

### File Tree Behavior

- `plan.md` is surfaced as the canonical planning artifact.
- Episode assets are listed beneath the plan.
- File icons follow VS Code-style language/file semantics.
- The selected artifact is highlighted in the tree.

### Special Renderers

The artifact viewer has specialized renderers for known artifact families:

- `assembly_definition.yaml` with electronics content renders a circuit schematic and wire view alongside the raw YAML,
- `simulation_result.json` renders stress and fluid summaries alongside the raw JSON,
- timeline assets render as circuit or event timelines.

The viewer should prefer purpose-built renderers over raw text when a specialized artifact exists.

### Code Steering

Clicking a code line adds that line, or line range, to the selected context.
This is part of the steering contract and must remain visible to the user.

## 11. 3D And Simulation Viewer

`DesignViewer` and `ModelViewer` are responsible for spatial inspection.

The viewport is not a CAD authoring tool.
It renders backend-provided artifacts:

- GLB model assets,
- simulation video,
- stress heatmaps,
- electronics overlays.

The default view is the 3D model unless a simulation video is present, in which case the video view takes precedence.

### Viewer Modes

The viewer supports four modes:

- 3D model,
- simulation video,
- stress heatmap,
- electronics.

Mode selection is explicit and user-controlled.

### Topology Browser

The topology browser is always tied to the loaded model.

- It can search parts,
- it can hide or reveal nodes,
- it can expose face, part, and subassembly selection modes,
- it can collapse and reopen,
- it can reset the camera.

Selecting a node or mesh adds CAD context to the episode.
The selected geometry should carry metadata such as node identity, topology level, and local hit information when available.

### Electronics Overlay

When circuit data is available, the viewer can render wires and electronics overlays.
That view is diagnostic, not schematic-authoring.

### Empty States

If no model assets are available:

- the viewer shows an explicit rebuild prompt,
- the rebuild action remains visible and actionable,
- the user is not left with a blank viewport and no explanation.

## 12. Feedback And Review

Feedback is a first-class part of the dashboard.

`FeedbackSystem` provides a trace-scoped modal with:

- thumbs up and thumbs down,
- common failure-topic chips,
- a free-text comment field,
- submit and cancel actions.

The feedback payload is episode-scoped and trace-scoped.
It should remain attributable to the exact output the user reacted to.

The default topic set covers the common failure modes surfaced by the product:

- misinterpretation,
- does not follow instructions,
- technical error,
- poor design,
- other.

Feedback submission should confirm success visibly and should not disappear silently on failure.

## 13. Theme And Visual Language

The dashboard uses a technical, high-density visual language.

### Theme

- Light and dark modes are both supported.
- Theme choice persists in local storage.
- The default experience should remain legible in both themes.

### Palette

- primary: blue,
- success: green,
- warning: amber,
- failure: red,
- neutral surfaces: slate/gray.

The palette should avoid purple-led defaults and avoid decorative gradients that weaken the engineering feel.

### Typography And Density

- Use compact labels and tight spacing for operational chrome.
- Use monospaced treatment for ids, line numbers, token counts, and numeric telemetry.
- Use larger type only for titles and critical state banners.

### Iconography

- File and language icons follow VS Code-style semantics.
- Action and status icons come from `lucide-react`.
- The icon set should communicate state before the user reads the copy.

### Motion

Motion should be functional:

- reveal state changes,
- emphasize live updates,
- support collapsible reasoning and context,
- avoid ornamental animation loops that do not carry meaning.

## 14. Responsive And Accessibility Requirements

The dashboard is desktop-first.
It must remain usable on laptops and narrower desktop widths, but it is not a mobile-native product.

Responsive behavior should preserve the following:

- the sidebar remains accessible,
- the current episode remains visible,
- the chat composer remains reachable,
- the artifact viewer remains usable,
- the viewport never traps the user with no path back to the episode controls.

Accessibility requirements:

- all icon-only actions need labels or accessible titles,
- keyboard navigation must reach all primary actions,
- focus states must remain visible,
- hover must never be the only path to a critical action,
- empty, offline, and failed states must be explicit and readable.

## 15. Data And Contract Boundaries

The frontend consumes generated OpenAPI clients and controller-backed polling and event updates.
It must stay aligned with the backend episode model and trace model rather than inventing client-side parallel state.

Contract boundaries:

- episode identity comes from the controller,
- trace ordering comes from persisted backend data,
- assets resolve through controller episode asset endpoints,
- WebSockets are deferred to a later phase; the MVP transport contract uses controller APIs plus polling/event refreshes,
- selected episode state may be restored from local storage,
- panel sizes may be restored from local storage,
- reasoning visibility may be restored from local storage or UI settings,
- the frontend must never directly access worker filesystems.

The frontend must treat the controller as the source of truth for all episode state transitions.

## 16. Non-Goals

The dashboard is not responsible for:

- writing CAD code,
- running physics simulation,
- pricing parts,
- reading worker-local files directly,
- synthesizing trace rows,
- replacing the controller with client-side logic,
- becoming a general-purpose IDE.

## 17. Acceptance Criteria

The frontend spec is satisfied when the dashboard can:

- switch between engineer and benchmark episodes from the sidebar,
- create new sessions from the current route,
- show live trace updates and a stop control while an episode runs,
- expose plan confirmation and request-change controls when a plan is ready,
- render reasoning, tool activity, errors, and compaction events from persisted traces,
- surface a telemetry-missing warning when reasoning is required but absent,
- support `@` mentions, code-line steering, and context-card removal,
- inspect code, YAML, JSON, simulation, and electronics artifacts in the artifact viewer,
- render GLB-backed 3D models, simulation videos, heatmaps, and electronics overlays,
- provide a visible feedback flow with score, topic, and comment,
- preserve theme, panel layout, and selected episode across refreshes,
- fail explicitly and readably when the backend is unreachable or assets are missing.

## 18. Deferred UI Refinement

The current UI is a workable baseline. The next frontend pass should refine hierarchy and panel integration rather than rethinking the core workflow.

Recommended later improvements:

- keep the Cursor/Windsurf-style chat interaction model,
- keep the left session history ledger,
- keep the colored file and artifact area,
- reduce the visual fragmentation in the evidence workspace,
- promote the active run summary above the chat as a stronger control surface,
- make the artifact panel feel more like a single inspection workspace and less like several independent widgets,
- use color as state encoding, not decoration,
- keep this as a later-phase polish pass so backend and workflow work stay prioritized.
