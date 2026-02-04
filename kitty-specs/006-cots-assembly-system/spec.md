# Feature Specification: COTS Assembly System

**Feature Branch**: `006-cots-assembly-system`
**Created**: 2026-02-01
**Status**: Draft
**Input**: User description: "Add an index that the model can query. A small search engine for parts... preview those parts via a tool call"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Find Standard Parts (Priority: P1)

As an autonomous agent, I want to search for standard mechanical parts by keyword (e.g., "NEMA 17", "608 bearing") so that I can find the correct component to include in my design without hallucinating dimensions.

**Why this priority**: Core functionality to prevent the agent from inventing non-existent parts, ensuring designs are realistic and manufacturable.

**Independent Test**: Can be tested by running the search tool with known keywords and verifying the returned list contains expected `bd_warehouse` components.

**Acceptance Scenarios**:

1. **Given** the COTS system is initialized with `bd_warehouse`, **When** the agent searches for "NEMA", **Then** the system returns a list of NEMA motor variants with their unique IDs and names.
2. **Given** the COTS system, **When** the agent searches for "Space Elevator", **Then** the system returns an empty list (or relevant semantic match if advanced) without crashing.

---

### User Story 2 - Inspect Part Details (Priority: P1)

As an autonomous agent, I want to preview a specific part to see its visual representation and read a text description of its orientation and ports, so that I can correctly align and fasten it in my assembly.

**Why this priority**: Knowing a part exists is insufficient; the agent needs to know where the mounting holes are and which way the shaft points to use it.

**Independent Test**: Can be tested by requesting a preview for a specific Part ID and verifying the output contains a valid image path and a structured text summary.

**Acceptance Scenarios**:

1. **Given** a valid Part ID (e.g., for a NEMA 17 stepper), **When** the agent requests a preview, **Then** the system returns a tuple containing:
    - A path to a rendered image (PNG/JPG) of the part.
    - A short text description (1-3 sentences) describing orientation (e.g., "Shaft points +Z").
    - A metadata dictionary (mass, torque, dimensions).

### Edge Cases

- **Invalid Part ID**: System returns a clear error message (not a traceback) if `preview_part` is called with a non-existent ID.
- **Missing Preview Assets**: If a part cannot be rendered (e.g., meshing failure), system returns a placeholder image and logs the error, rather than crashing.
- **Empty Search**: Searching for nonsense strings returns an empty list gracefully.
- **Large Result Sets**: Search results are truncated or paginated if a query matches >50 parts to prevent context window overflow.

---

### User Story 3 - Extensible Part Library (Priority: P2)

As a developer, I want the system to support pluggable part providers so that I can eventually add custom parts or imported STEP files beyond `bd_warehouse`.

**Why this priority**: `bd_warehouse` is the MVP, but the system must handle custom domain-specific parts (like springs) in the near future.

**Independent Test**: Can be tested by implementing a mock `PartProvider` that returns a dummy part and verifying it appears in search results and previews.

**Acceptance Scenarios**:

1. **Given** a new `PartProvider` is registered, **When** the registry is queried, **Then** parts from the new provider are included in the search index.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST provide a `search_parts(query: str) -> List[PartSummary]` tool exposed to the Agent.
- **FR-002**: The system MUST provide a `preview_part(part_id: str) -> PartPreview` tool exposed to the Agent.
- **FR-003**: The `PartPreview` output MUST include a rendered image of the part (isometric view preferred).
- **FR-004**: The `PartPreview` output MUST include a structured text description (max 3 sentences) describing key geometric features (orientation, ports).
- **FR-005**: The `PartPreview` output MUST include a metadata dictionary containing physical properties (mass, material) if available.
- **FR-006**: The system MUST index parts from the `bd_warehouse` library (specifically Motors, Beams, Bearings, Fasteners).
- **FR-007**: The system MUST explicitly exclude "Pipes", "Flanges", and "Gears" from the initial `bd_warehouse` index (as per scope).
- **FR-008**: The system MUST use an extensible `PartProvider` interface to allow future addition of non-`bd_warehouse` parts.
- **FR-009**: The system MUST generate Python import code or a `build123d` recipe snippet for the agent to instantiate the selected part.
- FR-010: The system MUST log all search queries and preview requests, including agent "thoughts" if provided, to the primary database for observability.
- FR-011: The system MUST fail fast and return clear error messages for invalid inputs or asset generation failures.

### Key Entities

- **Part**: Represents a specific COTS component. Attributes: `id` (unique), `name`, `provider`, `metadata` (JSON), `factory_function` (callable or import path).
- **PartProvider**: Abstract base class for sources of parts (e.g., `BDWarehouseProvider`, `LocalStepProvider`).
- **PartIndex**: Central registry/search engine that aggregates parts from all providers.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: `preview_part` returns results (Image + Text) in under 3 seconds for standard `bd_warehouse` parts.
- **SC-002**: Search recalls 100% of relevant `bd_warehouse` parts when queried by their exact class name (e.g., "Nema17").
- **SC-003**: The text description for a part successfully identifies the primary axis (e.g., "shaft along Z") for 100% of tested motor samples.
- **SC-004**: The system can successfully instantiate the `build123d` object for any selected part using the code snippet provided by the tool.
