---
name: render-evidence
description: Shared render-evidence workflow for Problemologist agents. Use when previewing scenes with `preview(...)` for live scene inspection or `preview_drawing()` for drafting packages, inspecting render or video media through the runtime's visual-inspection path, selecting benchmark/engineer/final render bundles, or resolving a screen-space pixel to a world-space hit with bundle-local render-query helpers such as `pick_preview_pixel(...)` and `query_render_bundle(...)`.
---

# Render Evidence

## Overview

Use render artifacts as evidence, not as filenames or summaries. This skill tells agents when to materialize a new preview, when to inspect existing media, how to select the right bundle or revision, and how to perform point-pick queries against immutable bundle snapshots.

The concrete media-inspection tool name depends on the backend. In controller/API-backed runs, use the controller-provided media-inspection tool. In Codex CLI-backed runs, use the visual-inspection path the runtime exposes instead of assuming the controller tool name exists.

## Read This First

- `references/bundle-history.md` when you need to select or verify the correct render bundle.
- `references/media-inspection.md` when you need to inspect images or videos.
- `references/point-pick.md` when you need a world-space hit or bundle-local query result from a pixel.

## Core Workflow

1. Decide whether you need a fresh render, existing evidence inspection, or a point-pick query.
2. Select a single bundle and revision for the judgment. Do not mix benchmark, engineer, and final bundles in one decision.
3. If the view or modality does not exist yet, call `preview(...)` for scene previews or `preview_drawing()` for drafting packages to materialize it.
4. If media already exists, inspect the artifact path itself with the runtime's media-inspection tool.
5. If the task asks what is under a pixel or how a click maps into world space, use the bundle-local render-query helpers against the immutable bundle snapshot.
6. Record the bundle identity, revision, view index, and artifact path in notes or review artifacts when evidence matters.

## Preview Contract

- Use `preview(...)` for live scene, objective-overlay, or render-bundle preview evidence.
- Use `preview_drawing()` for drafting packages and technical-drawing evidence.
- Do not treat `preview(...)` and `preview_drawing()` as interchangeable.

## Bundle Rules

- Prefer the latest revision only when the task is revision-scoped.
- For history or replay, resolve the bundle from the append-only render index and then read its bundle-local manifest.
- Treat `renders/render_manifest.json` as compatibility plumbing only when the runtime still exposes it.
- Never infer coordinates from raw pixels, filenames, or stale manifests.

## Inspection Rules

- Use RGB for general scene understanding.
- Use depth for clearance, collision, and spatial separation.
- Use segmentation for identity, overlap, and repeated-instance checks.
- Inspect the actual image or video path, not a directory listing or text summary.
- For motion evidence, inspect the first useful frames before changing geometry.
- If the view could be front or rear, confirm camera orientation before assuming a mirror flip.

## Point-Pick Rules

- Query only the bundle-local scene snapshot that produced the render.
- Keep bundle identity, revision, and scene hash aligned with the queried artifact.
- Return or record bundle identity, view identity, pixel coordinates, ray data, world point, hit state, and object identity if the helper provides them.
- Fail closed if the requested bundle, snapshot, or revision does not match the artifact on disk.

## Common Failure Modes

- Looking at the wrong revision.
- Reading filenames instead of pixels.
- Mixing benchmark, engineer, and final render buckets.
- Assuming a rear view is a mirrored front view.
- Treating inspection summaries as proof without looking at media.
- Using point-pick queries on stale bundles or mismatched snapshots.

## Role Notes

- Use this skill from benchmark planner, benchmark coder, benchmark reviewer, engineering planner, engineering coder, engineering plan reviewer, and engineering execution reviewer when render evidence is involved, including `preview(...)` and `preview_drawing()` outputs.
- Load it before making a render-based judgment or before adding render-specific guidance to a handoff.
- Keep backend-specific tool names out of Codex CLI-facing instructions unless the runtime actually exposes them.
