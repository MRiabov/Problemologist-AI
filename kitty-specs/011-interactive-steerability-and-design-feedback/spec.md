# Feature Specification: Interactive Steerability and Design Feedback

**Feature Branch**: `011-interactive-steerability-and-design-feedback`  
**Created**: 2026-02-15  
**Status**: Draft  
**Input**: User description: "Feature 'steerability'/'adaptability' (steerability is more correct). @roadmap.md package 4 explains it, there was also a research done on how the current version of it is implemented'/home/maksym/Work/proj/Problemologist/Problemologist-AI/specs/wp4_research_report.md. There is also, probably most importantly, lines 1016 to 1100 or so in @specs/desired_architecture.md about 'steerability'. We need everything listed in the desired architecture document about steerability."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Exact Pointing Feedback (Priority: P1)

An engineer selects a specific geometric feature (face, edge, or vertex) on a 3D part in the CAD viewer and provides a correction. The system automatically determines the best viewing angle, highlights the feature, and provides structured metadata to the agent.

**Why this priority**: Core of steerability; allows precise geometric intervention which is currently difficult with text only.

**Independent Test**: User can select a face, edge, or vertex in the CAD viewer. The agent's next turn includes a YAML-formatted list of selections with geometric metadata (centers, normals, directions) and a screenshot from an automatically selected isometric view showing the highlighted feature.

**Acceptance Scenarios**:

1. **Given** a rendered 3D model in the CAD viewer, **When** the user clicks a face, edge, or vertex, **Then** the feature is visually highlighted.
2. **Given** one or more selected features, **When** the user submits a prompt, **Then** the system automatically selects the best viewing angle (from 8 standard isometric views) and captures a snapshot where the selected parts are rendered more brightly.
3. **Given** a submitted prompt with selections, **When** the agent receives the turn, **Then** the context contains a YAML-like block with:
    - **Faces**: index, center, and normal.
    - **Edges**: center, direction, and arc/linear classification.
    - **Vertices**: world position.

---

### User Story 2 - Targeted Code Steering (Priority: P2)

An engineer references specific lines in the assembly or model code to request a modification (e.g., "@model.py:45-50: change the bracket thickness to 5mm").

**Why this priority**: Efficiency; prevents the agent from searching the whole file when the user knows exactly where the change is needed.

**Independent Test**: Typing `@filename:line-range` in the chat resolves the reference and passes the specific code snippet to the agent as high-priority context.

**Acceptance Scenarios**:

1. **Given** a chat input containing `@filename:L1-L2`, **When** the message is sent, **Then** the agent's prompt is enriched with the exact text content of those lines.
2. **Given** an invalid line range or filename, **When** the user sends the message, **Then** the system provides immediate feedback about the broken reference.

---

### User Story 3 - Graceful Interaction Queuing (Priority: P2)

An engineer notices the agent is heading in the wrong direction during a multi-turn reasoning process and provides a correction before the agent finishes its current block.

**Why this priority**: Reduces wasted tokens and time by allowing "mid-flight" course correction.

**Independent Test**: Send a message while the agent is executing a tool call; the message is queued and automatically delivered as the next prompt without manual re-entry.

**Acceptance Scenarios**:

1. **Given** the agent is in "Thinking" or "Executing tool" state, **When** the user sends a message, **Then** the UI shows the message as "Queued for next turn".
2. **When** the current agent execution block completes, **Then** the queued message is automatically dispatched as the next user input.

---

### User Story 4 - BOM @-mentions (Priority: P3)

An engineer @-mentions a part or subassembly from the Bill of Materials (BOM) to focus the agent's attention on it.

**Why this priority**: Convenience; simplifies referencing parts that have complex names or are deep in the hierarchy.

**Independent Test**: Using `@PartName` in chat resolves to the specific part metadata in the agent's context.

**Acceptance Scenarios**:

1. **Given** an assembly tree (BOM), **When** the user types `@` followed by a part name, **Then** the system provides an autocomplete list of parts.
2. **Given** a selected part via @-mention, **When** the prompt is sent, **Then** the agent receives the unique identifier and metadata for that part.

---

### Edge Cases

- **Topological Instability**: If the user selects `face_12` but a previous turn changed the geometry, the indices might have shifted. The system must handle or warn about stale references.
- **Multiple Selections**: Handling cases where the user selects multiple faces across different parts.
- **Overlapping References**: User @-mentions a part and also selects a face on it; system must deduplicate or correlate these references.
- **Network Interruptions during Queuing**: Ensuring the queue is persistent enough to handle page refreshes or transient disconnects.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST integrate `three-cad-viewer` to enable topological selection (faces, edges, volumes).
- **FR-002**: System MUST capture and transmit "visual context" (screenshots with highlights). The system MUST automatically select the optimal viewing angle from 8 isometric views to showcase the selected features.
- **FR-003**: System MUST parse `@filename:line-range` syntax in the chat interface and retrieve the corresponding source code.
- **FR-004**: System MUST provide an autocomplete mechanism for @-mentioning parts from the active assembly BOM.
- **FR-005**: System MUST implement a server-side (or persistent client-side) queue for user prompts sent during agent execution.
- **FR-006**: The Agent MUST be equipped with tools to translate user face selections into robust semantic selectors (e.g. `faces().sort_by(Axis.Z)[-1]`). Selection metadata MUST be provided in a YAML-like format including centers, normals, and directions.
- **FR-007**: System MUST highlight the referenced code or parts in the UI when the agent acknowledges them in its response.
- **FR-008**: System MUST support per-user steerability memory, allowing the agent to remember and apply user-specific design preferences across multiple turns or sessions.

### Key Entities *(include if feature involves data)*

- **GeometricSelection**: Represents a user's topological selection (part_id, face_idx, normal, center).
- **SteerablePrompt**: A wrapper for user messages that includes attachments (CodeReference, GeometricSelection, PartReference).
- **TurnQueue**: A FIFO queue for user inputs that are pending delivery to an active agent session.
- **UserSteeringPreference**: Persistent data capturing a user's specific feedback patterns or preferred design constraints.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of topological selections result in the correct feature metadata being transmitted to the agent.
- **SC-002**: Agent successfully identifies or generates a robust semantic selector for a selected face in 90% of verified test cases.
- **SC-003**: Queued messages are delivered to the agent's input stream within 500ms of the previous turn's completion.
- **SC-004**: Users report a 30% reduction in "re-prompting" effort when using targeted code steering compared to full-file context.
