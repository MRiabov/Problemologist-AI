# Implementation Plan: Agentic CAD Dashboard

*Path: kitty-specs/007-agentic-cad-dashboard/plan.md*

**Branch**: `007-agentic-cad-dashboard` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/007-agentic-cad-dashboard/spec.md`

## Summary

Implement the **Agentic CAD Dashboard** as a standalone **React/Vite** application. This replaces the previous Streamlit proposal. The dashboard consumes the Controller's OpenAPI to provide type-safe interfaces for monitoring runs, viewing 3D assets, and managing benchmarks.

## Technical Context

**Language/Version**: TypeScript 5+
**Frameworks**:

- `React`: UI Library.
- `Vite`: Build tool.
- `TanStack Query`: Data fetching.
- `react-three-fiber`: 3D Rendering.
**Dependencies**:
- `openapi-typescript`: Type generation.
**Infrastructure**:
- Deployed to Vercel/Railway.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Premium UI goals.]

## Project Structure

### Documentation

```
kitty-specs/007-agentic-cad-dashboard/
├── plan.md              # This file
├── research.md          # Research
├── data-model.md        # API Contract
└── tasks.md             # Tasks
```

### Source Code

```text
src/frontend/
├── src/
│   ├── components/      # UI Atoms/Molecules
│   ├── features/        # Business Logic (RunDetail, BenchmarkWizard)
│   ├── api/             # Generated Client
│   └── App.tsx
├── package.json
└── vite.config.ts
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| React/Vite | Interactivity & Polish. | Streamlit is great for prototypes but fails to deliver "Premium" custom layouts and rich 3D interactions. |
| Auto-generated Types | Reliability. | Manual/Any types lead to frontend bugs when backend schema changes. |
