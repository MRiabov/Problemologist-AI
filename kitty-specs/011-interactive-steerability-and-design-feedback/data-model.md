# Data Model: Steerability & Design Memory

## Entities

### GeometricSelection (Pydantic Model)
Represents a user's selection in the CAD viewer.
- `level`: `SelectionLevel` (FACE, EDGE, VERTEX, PART, SUBASSEMBLY)
- `target_id`: `str` (Face index, Part ID, or Subassembly ID)
- `center`: `Vector3` (World space)
- `normal`: `Optional[Vector3]` (For faces)
- `direction`: `Optional[Vector3]` (For edges)
- `bounding_box`: `Optional[AABB]`

### SteerablePrompt (API Model)
A user message enriched with steering context.
- `text`: `str` (The prompt)
- `selections`: `List[GeometricSelection]`
- `code_references`: `List[CodeReference]` (path, start_line, end_line)
- `mentions`: `List[str]` (Part/Subassembly IDs)

### TurnQueue (In-Memory)
- `session_id`: `str`
- `items`: `Queue[SteerablePrompt]`

### UserSteeringPreference (Database/Postgres)
Persistent user-specific design memory.
- `user_id`: `str`
- `preference_key`: `str` (e.g., "preferred_fastener_type")
- `preference_value`: `JSON`
- `last_updated`: `DateTime`

## Relationships
- A `SteerablePrompt` contains multiple `GeometricSelection` and `CodeReference`.
- `UserSteeringPreference` is linked to the user's global profile and is injected into the system prompt during planning.
