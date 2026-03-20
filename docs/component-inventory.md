# Problemologist-AI - Component Inventory

**Date:** 2026-03-20

## Scope

This is a secondary UI reference for the React frontend in `frontend/src/`. Use it when you are changing the dashboard; skip it for backend-only work. The dashboard is intentionally agent-centric: the core experience is observing episodes, traces, assets, and simulation outputs rather than editing CAD directly in the browser.

## Top-Level UI Surfaces

| Surface | Key Files | Responsibility |
| --- | --- | --- |
| App bootstrap | `frontend/src/main.tsx`, `frontend/src/App.tsx` | Wires providers, router, and the API base URL |
| Shell layout | `frontend/src/components/layout/AppLayout.tsx`, `frontend/src/components/layout/Sidebar.tsx` | Provides the fixed app shell, navigation, history, and feedback entry points |
| Engineer workspace | `frontend/src/pages/EngineerWorkspace.tsx` | Main solving view for agent runs |
| Benchmark pipeline | `frontend/src/pages/BenchmarkGeneration.tsx` | Benchmark generation and review view |
| Settings | `frontend/src/pages/Settings.tsx` | Theme and workspace preferences |
| Shared workspace shell | `frontend/src/components/workspace/UnifiedGeneratorView.tsx` | Common split-panel layout used by both workspace pages |

## App Shell And Routing

### `frontend/src/main.tsx`

- Creates the React root.
- Sets `OpenAPI.BASE` from `VITE_API_URL` when present.
- Wraps the app in `ThemeProvider`, `UISettingsProvider`, `ConnectionProvider`, and `EpisodeProvider`.
- Mounts the router under `BrowserRouter`.

### `frontend/src/App.tsx`

- Routes `/` to `EngineerWorkspace`.
- Routes `/benchmark` to `BenchmarkGeneration`.
- Routes `/settings` to `Settings`.
- Uses `AppLayout` as the shared outer shell.

### `AppLayout` and `Sidebar`

- `AppLayout` keeps the shell stable for tests and opens the global feedback modal when a trace is selected.
- `Sidebar` provides the primary navigation, episode search/filtering, episode history, create-new actions, and thumbs up/down feedback shortcuts.
- The sidebar is the main launch point for switching between engineer and benchmark sessions.

## Core Workspace Components

### `UnifiedGeneratorView`

- The central shared layout for both main pages.
- Uses `ResizablePanelGroup` to divide the screen into chat, viewport, and artifact areas.
- Reads the selected episode from `EpisodeContext`.
- Resolves asset URLs through the controller episode asset endpoint.
- Parses `assembly_definition.yaml` to expose electronics content to the viewer.

### `ChatWindow`

- Renders the trace timeline and the prompt composer for the active episode.
- Handles run creation, continuation, steering, benchmark confirmation, and simulation triggers.
- Shows context usage, reasoning visibility toggles, and execution-plan confirmation UI.
- Uses `ConnectionError` when the backend is unavailable.

### `TraceList`

- Renders traces into three main forms:
  - reasoning spans
  - tool/action cards
  - error and event banners
- Hides reasoning spans unless the user enables them or the episode requires them.
- Surfaces feedback controls on the latest LLM output.

### `ArtifactView`

- Presents the file tree and the currently selected artifact.
- Has special renderers for:
  - `assembly_definition.yaml` with electronics content
  - `simulation_result.json`
  - circuit data assets
  - timeline data assets
- Connects code and artifact highlights back into the episode context.

### `ChatInput`

- Supports plain prompts, `@` mentions, and line-range references such as `@file.py:L10-L20`.
- Builds metadata payloads for code references, CAD selections, and context items.
- Lets users add parts or files into the active context from typed mentions.

### Other Workspace Pieces

- `ObjectivesForm` captures benchmark objectives.
- `ContextCards` shows selected files, parts, and references.
- `ActionCard` renders tool-call details.
- `ThoughtBlock` renders reasoning spans.
- `HighlightedContent` formats markdown and inline references.
- `FeedbackSystem` captures episode feedback and forwards it to the controller.

## Visualization Stack

### `DesignViewer`

- Switches between 3D, simulation video, stress heatmaps, and electronics views.
- Defaults to video when a rendered simulation exists.
- Uses `ModelViewer`, `CircuitSchematic`, `WireView`, and heatmap overlays for the available artifacts.

### Supporting Visualization Components

- `ModelViewer` renders the 3D model and topology overlays.
- `ModelBrowser` provides topology node structures for part selection and context injection.
- `CircuitSchematic` renders circuit/electronics layouts.
- `WireView` traces wire routing over the model.
- `CircuitTimeline` shows time-based circuit or simulation events.
- `SimulationResults` renders stress and fluid metrics in a readable summary format.

## State And Data Flow

| Area | Key Files | Notes |
| --- | --- | --- |
| Episode state | `frontend/src/context/EpisodeContext.tsx` | Owns selected episode, episode polling, creation flows, continuation, steerability, and context selection |
| Connection state | `frontend/src/context/ConnectionContext.tsx` | Tracks backend connectivity and mock mode |
| Theme state | `frontend/src/context/ThemeContext.tsx` | Handles light/dark theme switching |
| UI preferences | `frontend/src/context/UISettingsContext.tsx` | Stores reasoning visibility and related workspace preferences |
| API client | `frontend/src/api/client.ts`, `frontend/src/api/generated/` | Wraps the generated controller client and the manual helpers used by the UI |

## Shared UI Primitives

The frontend uses a small component library under `frontend/src/components/ui/`:

- `button`
- `card`
- `input`
- `tabs`
- `sheet`
- `scroll-area`
- `resizable`
- `separator`
- `badge`

These primitives are used throughout the shell and workspace layouts, so visual changes usually cascade through the entire dashboard.

## Testing Anchors

| Area | Test Files |
| --- | --- |
| App shell and workspaces | `frontend/src/pages/__tests__/EngineerWorkspace.test.tsx`, `frontend/src/pages/__tests__/BenchmarkGeneration.test.tsx` |
| Chat and reasoning | `frontend/src/components/Chat/__tests__/ChatInput.test.tsx`, `frontend/src/components/workspace/__tests__/ChatWindow.test.tsx`, `frontend/src/components/workspace/__tests__/ThoughtBlock.test.tsx` |
| Feedback and actions | `frontend/src/components/workspace/__tests__/FeedbackSystem.test.tsx`, `frontend/src/components/workspace/__tests__/ActionCard.test.tsx` |
| API client behavior | `frontend/src/api/__tests__/client.test.ts` |

## Practical Notes

- `EngineerWorkspace` and `BenchmarkGeneration` intentionally share the same shell. When changing layout behavior, update `UnifiedGeneratorView` first.
- `DesignViewer` chooses the visible mode based on the artifacts currently attached to the episode.
- Most workspace interactions are episode-scoped, so the selected episode in `EpisodeContext` is the main source of truth.
- The dashboard relies on generated OpenAPI types, so controller API changes should be reflected in `frontend/src/api/generated/`.
