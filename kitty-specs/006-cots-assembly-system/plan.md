# Implementation Plan: COTS Assembly System

*Path: kitty-specs/006-cots-assembly-system/plan.md*

**Branch**: `006-cots-assembly-system` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/006-cots-assembly-system/spec.md`

## Summary

Implement the **COTS Assembly System** as a SQL-backed catalog. Create a build-time **Indexer** that extracts metadata from `bd_warehouse` into a SQLite database. Implement a **Search Tool** (or Subagent) that allows the Engineer Agent to query this database using SQL or structured filters to find specific parts and their usage code.

## Technical Context

**Language/Version**: Python 3.10+
**Dependencies**:

- `bd_warehouse`: Source of parts.
- `sqlite3`: Catalog storage.
- `build123d`: For part instantiation during indexing.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Precision goals.]

## Project Structure

### Documentation

```
kitty-specs/006-cots-assembly-system/
├── plan.md              # This file
├── research.md          # Research
├── data-model.md        # DB Schema
└── tasks.md             # Tasks
```

### Source Code

```text
src/
├── cots/
│   ├── indexer.py       # Build-time scraper
│   ├── database/        # Schema definition
│   └── runtime.py       # Search Tool logic
└── worker/
    └── data/
        └── parts.db     # The artifact
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| SQL Catalog | Exact matching. | Vector search is fuzzy and often fails on specific dimensions (e.g., specific screw lengths). |
| Build-time Indexing | Runtime performance. | Instantiating 1000s of `bd_warehouse` parts at runtime to check properties is too slow. |
