# Feature Specification: COTS Assembly System

**Feature**: 006-cots-assembly-system
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **COTS (Commercial Off-The-Shelf) Assembly System** is a searchable parts catalog that allows agents to discover and utilize standard components (motors, fasteners, bearings).

To avoid context flooding and hallucination, this system is exposed via a dedicated **Search Subagent** (a specific node in `deepagents` or a specialized tool). This agent queries a **SQL Database** of part metadata rather than performing vector searches, ensuring precise specification matching.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Exact Specification Matching**: Enable agents to find parts with specific properties (e.g., "NEMA 17 motor with > 0.4Nm torque").
2. **Code-First Integration**: Provide `build123d` recipes for every part so agents can instantiate them immediately.
3. **Low Latency**: Subagent returns results < 2s.

### 2.2. Success Criteria

- **Precision**: SQL queries return exact part variants (e.g., specific screw length).
- **Coverage**: Indexes 100% of the `bd_warehouse` library.

## 3. User Stories

- **As an Agent**, I want to tell the Search Subagent "Find me a M3 screw, 10mm long" and get back the exact import code.
- **As an Agent**, I want to verify if a "NEMA 23" fits in my design by asking for its dimensions.

## 4. Functional Requirements

### 4.1. The Search Subagent

The Planner or Engineer agent delegates to the Search Subagent.

- **Input**: Natural language query (e.g., "High torque stepper motor").
- **Process**:
    1. Subagent converts query to SQL (e.g., `SELECT * FROM parts WHERE type='motor' AND torque > 0.5`).
    2. Executes query against `parts.db` (SQLite/DuckDB bundled in container).
    3. Selects top 3 matches.
- **Output**: List of matches with:
  - `Part ID`
  - `Description`
  - `Python Usage Snippet`

### 4.2. Database Schema

- `parts`: id, name, category, properties (JSON), usage_code.

### 4.3. Assets

- **Preview Images**: Pre-rendered thumbnails for Human interaction (Dashboard).
- **Metadata**: Extracted from `bd_warehouse` at build time.

## 5. Technical Design

### 5.1. Tech Stack

- **Database**: SQLite (read-only in Worker).
- **Source**: `bd_warehouse` (Python library).
- **Indexer**: A build-time script that scrapes `bd_warehouse` and populates `parts.db`.

### 5.2. Integration

- **Worker Node**: Contains `parts.db`.
- **Agent**: Contains a "Search Parts" tool that effectively acts as the interface to the Subagent/SQL logic.
