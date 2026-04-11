---
title: Publication Shape Repository Pruning Migration
status: investigation
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - electronics_planner
  - engineer_plan_reviewer
  - engineer_coder
  - electronics_reviewer
  - engineer_execution_reviewer
  - skill_agent
  - cots_search
added_at: '2026-04-11T00:00:00Z'
---

# Publication Shape Repository Pruning Migration

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration defines the publication-shaped subset of Problemologist-AI for
a conference submission. The target is not the full brownfield repository; the
target is the smallest release bundle that can reproduce the paper's benchmark
generation, engineer-solution, simulation, render-evidence, and paper-figure
claims from a clean checkout.

The source of truth for the full system remains
`specs/desired_architecture.md` and the architecture files beneath
`specs/architecture/`. This migration narrows the default release surface to
the paths that the paper actually depends on and classifies the rest as
development-only bulk, optional extensions, or archival material.

The trimming rule is claim-driven: if the final camera-ready paper cites a
capability as part of the evaluated result set, the supporting code stays in
the publication bundle; if the capability is only future work, exploratory
research, or a convenience surface, it does not belong in the bundle.

## Problem Statement

The repository currently mixes three different shapes in one tree:

1. The backend runtime needed to generate benchmarks and solve them.
2. The supporting development and inspection surfaces that make the runtime
   easier to use.
3. The paper drafts, generated outputs, and historical experiments that are
   not required to reproduce the submitted claims.

That mix has already inflated the tree to a point where the source bundle is
no longer self-describing. The conference submission path should not depend on
the secondary dashboard, the static website mirror, generated logs, duplicate
paper formats, or old experimental scaffolding. Those surfaces are useful
during development, but they are not part of the publication contract.

The current shape also makes it unclear which features are core evidence for
the paper and which features are only extensibility. That ambiguity matters
because the repo still carries multiple auxiliary branches, duplicate
academic-submission artifacts, and broad test and dataset sets that are larger
than the paper's claim surface.

## Current-State Inventory

| Area | Current state | Publication decision |
| -- | -- | -- |
| `controller/`, `worker_light/`, `worker_heavy/`, `worker_renderer/`, `shared/`, `config/` | Core runtime, shared schemas, validation, simulation, and worker orchestration. | Keep, but remove dead branches and compatibility-only plumbing that are not needed for the paper claims. |
| `frontend/` | Secondary React dashboard and generated client bundle. | Exclude from the publication bundle unless a final paper figure or claim explicitly depends on it. |
| `website/` | Static marketing mirror. | Exclude from the publication bundle. |
| `docs/academic-submission/` | Mixed paper source, duplicate drafts, generated PDF/log outputs, figure assets, and reproducibility scripts. | Keep only the canonical paper source and the figure/repro assets cited by the paper; remove duplicate drafts and generated outputs from the bundle. |
| `dataset/data/` and `dataset/evals/` | Large seed and eval corpus, including render and handoff artifacts. | Keep only the curated publication seeds and eval rows; move the rest out of the default release surface. |
| `renders/`, `logs/`, `.tmp*`, caches, and `__pycache__/` | Generated runtime outputs and ephemeral state. | Exclude from source control and from the publication bundle. |
| `scripts/experiments/` and other one-off probes | Experimental harnesses and performance probes. | Archive or remove from the publication bundle unless they are explicitly cited in the paper. |
| `tests/e2e/` and broad historical integration coverage | Large verification surface covering features beyond the paper claims. | Curate down to the minimal paper-critical integration subset and archive the rest from the release path. |
| `specs/migrations/` and auxiliary notes under `specs/architecture/auxillary/` | Internal planning history and exploratory notes. | Keep for development history, but exclude from the publication bundle. |
| `docs/project-overview.md`, `docs/source-tree-analysis.md`, `docs/component-inventory.md`, `docs/development-guide.md`, `docs/deployment-guide.md` | Internal orientation and support docs. | Keep as development docs, but do not make them part of the conference submission contract. |

## Proposed Target State

1. The publication bundle is a backend-first release slice that contains only
   the code required to build, run, and verify the paper's benchmark
   generator, engineer solver, simulation/render validation, and paper
   figures.
2. The release slice is driven by a claim matrix derived from the camera-ready
   paper. A capability stays in the bundle only when the paper cites it as
   part of the evaluated result set.
3. The dashboard and website are not required to build or validate the
   publication bundle.
4. Generated outputs, logs, caches, render bundles, and other ephemeral
   runtime artifacts are not versioned as source.
5. The academic-submission tree has one canonical paper source and one
   canonical reproducibility path. Duplicate drafts, exported PDFs, logs, and
   alternate paper formats are outside the bundle.
6. The verification surface is reduced to the minimal paper-critical
   integration and seed set. It still proves the benchmark generator,
   engineer solver, render evidence, and handover contracts that the paper
   claims.
7. Optional extensions remain available only if they are not required to
   reproduce the published claims. They do not participate in the default
   conference submission path.

## Required Work

### 1. Define the publication boundary

- Create a publication claim matrix from
  `docs/academic-submission/final-project-report.tex` and use it to classify
  every major repo surface as core, supporting, optional, or archive.
- Add a publication manifest or equivalent release profile that enumerates the
  paths included in the submission bundle.
- Make the release scripts consume the manifest instead of implicitly
  packaging the full repository tree.
- Keep the manifest fail-closed: anything not explicitly included is not part
  of the publication bundle.

### 2. Collapse the academic-submission tree

- Keep one canonical LaTeX source for the paper and the minimal bibliography
  and figure inputs it depends on.
- Preserve the reproducible benchmark figure sources under
  `docs/academic-submission/paper_benchmark/` if the paper still cites them.
- Remove duplicate drafts, generated PDF/log files, and alternate document
  exports from the bundle.
- Demote any figure or matrix artifact that is not referenced by the final
  paper to archive-only status.

### 3. Prune secondary UI and marketing surfaces

- Remove `frontend/` from the publication bundle unless the final paper
  explicitly depends on a dashboard screenshot or interactive UI flow.
- Remove `website/` from the publication bundle.
- Update the docs and packaging references so the conference artifact does not
  imply that the UI or marketing mirror is required for reproduction.

### 4. Reduce generated and exploratory bulk

- Stop treating `renders/`, `logs/`, `.tmp*`, cache trees, and other runtime
  outputs as source.
- Move experiment harnesses and scratch utilities out of the default
  publication path if they are not directly cited by the paper.
- Remove dead compatibility shims and historical scaffolding from the
  publication bundle whenever they are not needed to reproduce the submitted
  claims.

### 5. Curate the verification surface

- Keep the integration tests and seed fixtures needed to prove the benchmark
  generator, engineer solver, simulation, render evidence, COTS selection,
  and handover contracts cited in the paper.
- Demote or archive UI-only, experiment-only, and legacy-contract tests that
  are not part of the publication claims.
- Ensure the publication bundle can run the paper-critical tests from a clean
  checkout without the secondary UI or website present.

### 6. Align docs with the trimmed bundle

- Update the repository overview docs so they distinguish the publication
  bundle from the full development tree.
- Update the academic-submission notes so they describe a single canonical
  paper source and the reproducible asset inputs.
- Remove or demote internal docs that are only useful for ongoing development
  and not for the conference artifact.

### 7. Freeze optional extension branches

- Keep only the modality branches that the final paper explicitly claims as
  evaluated results.
- Leave unclaimed branches behind an optional extension boundary so they do
  not inflate the default publication checkout.
- Treat anything described only as future work in the paper as out of bundle
  by default.

## Non-Goals

- No redesign of the controller, worker, or shared-contract architecture.
- No model-quality work or simulation-performance optimization.
- No attempt to delete all development history from the repo; the goal is to
  exclude it from the publication bundle, not erase it from source control
  history.
- No new benchmark families or new scientific claims beyond the paper.
- No rewrite of the paper narrative to justify keeping extra code.

## Sequencing

The safe order is:

1. Define the claim matrix and publication manifest.
2. Collapse the academic-submission tree to canonical sources.
3. Prune the secondary UI, website, and generated output surfaces from the
   default bundle.
4. Curate the evaluation, dataset, and integration-test surface down to the
   paper-critical subset.
5. Update docs and release scripts last.

## Acceptance Criteria

1. The repository contains a documented publication manifest that names the
   minimal files and directories required to rebuild the conference artifact.
2. A clean publication checkout does not require `frontend/` or `website/` to
   rebuild the paper-reproducible runtime or run the paper-critical tests.
3. The publication bundle contains one canonical paper source and no tracked
   generated PDF, log, or duplicate draft artifacts.
4. Generated outputs, caches, logs, and render bundles are excluded from the
   source release.
5. The curated test set covers the paper's benchmark generator, engineer
   solver, simulation, render evidence, and handover contract paths.
6. The publication bundle still satisfies the core architecture contracts for
   benchmark generation, engineer solving, simulation, rendering, and
   handover validation.

## Migration Checklist

### Bundle boundary

- [ ] Write the publication claim matrix from
  `docs/academic-submission/final-project-report.tex`.
- [ ] Classify every major repo surface as core, supporting, optional, or
  archive.
- [ ] Add a publication manifest or equivalent release profile.
- [ ] Wire the release scripts to the manifest so the bundle is fail-closed.

### Paper artifacts

- [ ] Collapse `docs/academic-submission/` to one canonical paper source and
  the minimal reproducibility inputs it needs.
- [ ] Preserve the reproducible benchmark figure sources under
  `docs/academic-submission/paper_benchmark/` only if the final paper still
  cites them.
- [ ] Remove generated PDF, log, and duplicate draft outputs from the
  publication bundle.
- [ ] Archive any unused alternate paper formats or draft notes.

### Runtime bundle trim

- [ ] Remove `frontend/` from the default publication bundle.
- [ ] Remove `website/` from the default publication bundle.
- [ ] Exclude generated artifacts, caches, logs, and render outputs from the
  source release path.
- [ ] Archive experiment harnesses and scratch utilities that are not cited by
  the paper.

### Verification and docs

- [ ] Curate the dataset and integration-test surface down to the
  paper-critical subset.
- [ ] Update the repository overview docs to distinguish the publication
  bundle from the development tree.
- [ ] Update the academic-submission notes to describe the canonical paper
  source and reproducible assets.
- [ ] Record the final publication bundle contents and the rationale for each
  excluded surface.

## File-Level Change Set

- `docs/academic-submission/final-project-report.tex`
- `docs/academic-submission/publication-manifest.md`
- `docs/academic-submission/Problemologist_AI_Research_Matrix.md`
- `docs/academic-submission/Foundations of AI - Final Project Report.md`
- `docs/academic-submission/final_project_report.md`
- `docs/academic-submission/final_project_report.pdf`
- `docs/academic-submission/final_project_report.log`
- `docs/academic-submission/Problemologist_AI_Research_Matrix.odt`
- `docs/academic-submission/IEEEtranBST2.zip`
- `docs/academic-submission/export_report.sh`
- `docs/academic-submission/render_sources/README.md`
- `docs/academic-submission/paper_benchmark/`
- `docs/project-overview.md`
- `docs/source-tree-analysis.md`
- `docs/backend-reference.md`
- `docs/index.md`
- `docs/spec-coverage.md`
- `docs/development-guide.md`
- `docs/deployment-guide.md`
- `config/prompts.yaml`
- `config/agents_config.yaml`
- `config/skills_config.yaml`
- `dataset/evals/`
- `dataset/data/seed/`
- `tests/integration/`
- `tests/e2e/`
- `scripts/experiments/`
- `frontend/`
- `website/`
