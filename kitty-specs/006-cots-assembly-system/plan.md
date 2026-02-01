# Implementation Plan: COTS Assembly System
*Path: kitty-specs/006-cots-assembly-system/plan.md*

**Branch**: `006-cots-assembly-system` | **Date**: 2026-02-01 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/006-cots-assembly-system/spec.md`

## Summary

Implement a **COTS (Commercial Off-The-Shelf) Assembly System** that allows the autonomous agent to search for and use standard mechanical parts (motors, bearings, fasteners) from the `bd_warehouse` library. This prevents the agent from hallucinating dimensions and ensures manufacturability. The system exposes two tools: `search_parts` and `preview_part`, backed by an in-memory index and an extensible `PartProvider` architecture.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: `bd_warehouse` (new), `build123d` (existing), `vtk` or `pyglet` (existing for render)
**Storage**: In-memory index (populated at startup), JSON file for part descriptions.
**Testing**: `pytest` for unit tests of providers and index.
**Target Platform**: Linux (Local execution)
**Project Type**: Python Library Module (`src/cots`)
**Performance Goals**: Search < 10ms, Preview generation < 3s.
**Constraints**: Must run locally without external API calls (except initial package install).

## Constitution Check

*Skipped: No project constitution found.*

## Project Structure

### Documentation (this feature)

```
kitty-specs/006-cots-assembly-system/
├── plan.md              # This file
├── research.md          # Technical decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Usage guide
├── contracts/           # Interface definitions
│   └── cots_interface.py
└── checklists/
    └── requirements.md
```

### Source Code (repository root)

```
src/
├── cots/
│   ├── __init__.py
│   ├── core.py           # PartIndex implementation
│   └── providers/
│       ├── __init__.py
│       ├── base.py       # PartProvider abstract base class
│       └── bd_warehouse.py # Adapter for bd_warehouse library
├── environment/
│   └── tools.py          # Update: register new tools
└── assets/
    └── cots_descriptions.json # Pre-generated descriptions
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Abstract Provider Pattern | Future extension for custom/STEP parts | Hardcoding `bd_warehouse` would require rewrite when adding new sources |
