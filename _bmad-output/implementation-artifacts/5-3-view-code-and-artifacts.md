# Story 5.3: View Code and Artifacts

Status: ready-for-dev

## Story

As a human operator, I want to view code, plans, manifests, logs, and other files in the browser so that I can inspect the actual artifacts that produced the run.

## Acceptance Criteria

1. Given a run has plan, TODO, code, or review files, when I open the artifact viewer, then the file tree preserves the workspace hierarchy and full relative paths, and line numbers plus syntax coloring are available.
2. Given a markdown, code, YAML, or JSON file is selected, when I inspect it, then the UI shows the exact persisted contents from the episode payload, not a synthesized placeholder or stale local copy.
3. Given a benchmark or solution run, when I inspect its artifacts, then I can see related manifests, review outputs, and validation logs in the same workspace view.
4. Given multiple artifacts share the same basename, when I browse or open them, then the UI distinguishes them by full path and opens the exact file that was clicked.
5. Given a tool row or inline mention points at a persisted artifact path, when I click it, then the artifact viewer opens the exact file rather than a sibling with a matching basename.
6. Given no artifact is selected, when the viewer loads, then it shows an explicit empty state or default artifact selection rather than a blank or misleading panel.

## Tasks / Subtasks

- [ ] Rebuild `ArtifactView` around a path-aware tree model so folder structure is preserved (`reviews/`, `.manifests/`, `renders/`, root files) and duplicate basenames remain unambiguous. (AC: 1, 3, 4, 6)
  - [ ] Keep `plan.md` surfaced first, but stop flattening every asset to `asset.s3_path.split('/').pop()`.
  - [ ] Use full relative path labels or breadcrumbs for tree rows and tooltips while keeping icon selection based on file type.
  - [ ] Keep the existing syntax-highlighter and line-number renderer for text/code/JSON/YAML files.
- [ ] Surface persisted validation logs as a read-only artifact in the browser workspace when `selectedEpisode.validation_logs` is present. Render the exact text from the episode payload and keep the failure banner as a secondary summary, not the only place logs exist. (AC: 2, 3, 6)
  - [ ] Do not invent blank logs or replace the current failure summary with a synthetic placeholder.
  - [ ] Use a deterministic virtual artifact key if a file-like entry is needed for the tree.
- [ ] Make artifact-opening path resolution exact in `ActionCard` and `HighlightedContent` so tool rows and inline file references resolve the intended file path before falling back to basename matching. (AC: 4, 5)
  - [ ] Preserve exact matching for `.manifests/` and `reviews/` entries.
  - [ ] Keep line-range mentions working for `@path:start-end` references.
- [ ] Add stable test selectors for the artifact tree and selected artifact panel so browser tests can assert the hierarchy without brittle text-only matching. (AC: 1-6)
  - [ ] Keep existing `code-line-#` selectors intact.
  - [ ] Add selectors that distinguish folder rows from leaf artifacts if the tree becomes nested.
- [ ] Extend live-browser integration coverage so a completed run proves the tree can open root code files plus nested review, manifest, and log artifacts from the live controller payload. (AC: 1-6)
  - [ ] Reuse `INT-164` for code-viewer / line-target coverage and extend it with path-aware file-tree assertions.
  - [ ] Add or extend a focused frontend slice for tool-row path jumping if the existing `INT-161` coverage is insufficient.

## Dev Notes

- Relevant architecture patterns and constraints:
  - The frontend is controller-first. Artifact browsing must come from persisted episode payloads (`EpisodeResponse.assets` and `EpisodeResponse.validation_logs`), not from direct worker filesystem access.
  - `ArtifactView` is the inspection surface for plan and episode assets. Keep it read-only and backed by persisted controller data instead of adding a parallel browser IDE.
  - Review manifests under `.manifests/` are system-owned in worker storage, but controller sync already surfaces them as episode assets. The UI must preserve their relative paths rather than flattening them.
  - `HighlightedContent` owns inline `@path:start-end` handling. Keep mention parsing consistent with exact path resolution so line-range context remains deterministic.
  - `ActionCard` is the tool-row affordance that jumps into artifacts. Keep the click-through behavior tied to the live asset list, not to synthetic file state.
  - `validation_logs` already exist on `EpisodeResponse`; surface them as a browsable artifact instead of inventing a second logging channel.
  - Keep syntax highlighting and line numbers. Use the existing `react-syntax-highlighter` path and the current file/language icon mapping instead of swapping to a new viewer stack.
  - The UI remains desktop-first and technical. Do not replace the artifact browser with a generic text dump or a browser-side file editor.

### Project Structure Notes

- `ArtifactView` is the single artifact browser. Keep file-tree logic, selected-file rendering, and the log/manifest surfaces in that component unless a smaller helper is clearly justified.
- `ActionCard` and `HighlightedContent` should share the same path-normalization logic so tool rows and inline mentions cannot drift apart.
- `UnifiedGeneratorView` should remain the shell/layout compositor unless a prop bridge is truly required for the artifact panel.
- Preserve the current failure banner in `ChatWindow` as a summary surface. If logs are also shown in the artifact panel, do not remove the banner.
- Current flat basename-based tree rendering is a known variance. Fix it by preserving relative paths, not by duplicating entries or hardcoding special cases.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 5: UI, Visualization, and Demo, Story 5.3]
- [Source: \_bmad-output/planning-artifacts/prd.md, MVP UI scope, artifact inspection, and debuggability requirements]
- [Source: \_bmad-output/planning-artifacts/ux-design-specification.md, sections 7, 10, 11, and 17]
- [Source: specs/frontend-specs.md, code viewer, CAD/simulation viewer, and shared workflow requirements]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, controller-first artifact surface and path-permission policy]
- [Source: specs/architecture/distributed-execution.md, controller-backed asset proxy contract]
- [Source: specs/architecture/observability.md, episode/asset/traces linkage and validation-log observability]
- \[Source: controller/api/routes/episodes.py, `EpisodeResponse` assets/traces/validation_logs payload and asset proxy\]
- \[Source: controller/agent/benchmark/graph.py, workspace asset sync for root files, reviews, and `.manifests/` artifacts\]
- [Source: frontend/src/components/workspace/ArtifactView.tsx, current file-tree and syntax-highlighter implementation]
- [Source: frontend/src/components/workspace/ActionCard.tsx, tool-row artifact jumping]
- [Source: frontend/src/components/workspace/HighlightedContent.tsx, inline path mention parsing]
- [Source: specs/integration-tests.md, INT-161 and INT-164]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
