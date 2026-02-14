# Desired Architecture: WP4 - Adaptability

## Objective of the system

To enable the agent to **modify existing designs** based on new requirements or human feedback, rather than regenerating from scratch. This is the "Human-in-the-loop" workflow. The goal is to make the AI a "Co-pilot" that can take direction, rather than a "Generator" that simply dumps a file and forgets it.

### Outputs and end goals

1. **Iterative Refinement**: The agent takes an existing CAD model + a "change request" and outputs a modified model.
2. **Semantic Selection**: The user (or agent) can refer to "the top face" or "the mounting holes" without knowing their internal B-Rep IDs or exact coordinates.
3. **Preservation of Intent**: Modifications should respect existing constraints (e.g., "Moves this hole 5mm right" should not break the hole's diameter or thread type).
4. **Visual Diff**: Presenting clear "Before vs After" comparisons to the user.

## The Adaptability Loop

We introduce a **Stateful Interaction** model. In standard generation, the conversation is stateless (Request -> Result). Here, we maintain a **Session Context**.

### Session Context (Microservice State)

We do not store session state in the Agent's memory. We use a **Redis-backed Session Store**.

* **Key**: `session:{session_id}`
* **Value**: JSON blob containing:
  * `current_script_path`: `/tmp/sessions/{id}/design_v1.py`
  * `semantic_map`: `{"face_42": "top_lid"}`
  * `history`: `["Created box", "Moved hole"]`

### The Workflow

### The Workflow

1. **Ingest**: The agent loads the existing script and the current B-Rep model.
2. **Annotate**: The agent runs a "Tagger" tool to understand the geometry (e.g., identifying "Top Face", "Side Wall", "Bolt Pattern").
3. **Interpret**: The agent parses the user's request: "Make the box 10mm taller."
4. **Locate**: The agent finds the relevant line in the python script.
    * *Search Strategy*: It looks for the definition of the base shape (e.g., `Box(length, width, height)`).
    * *Identification*: It identifies `height` as the parameter to change.
5. **Modify**: The agent edits the script using `edit_file` tools.
6. **Verify**: The specific change is verified.
    * *Assertion*: `assert new_height == old_height + 10`.
7. **Regress**: Use the standard verification suite to ensure nothing *else* broke.
8. **Present**: The system renders the new model and generates a Visual Diff.

## Semantic Selection & The "Mental Map"

We need a bridge between "Human Language" and "CAD Topology".

### The "Semantic Mapper" Tool

The agent uses a helper function (or sub-agent) to inspect the B-Rep model and tag features.

```python
# Conceptual Tool
def analyze_topology(compound):
    """Returns a list of significant features with generated names."""
    features = []
    # Find top-most face
    top_face = compound.faces().sort_by(Axis.Z)[-1]
    features.append({"id": top_face.id, "tag": "top_surface", "type": "Face"})
    
    # Find cylindrical holes
    holes = compound.faces().filter_by(GeomType.CYLINDER)
    for i, hole in enumerate(holes):
         features.append({"id": hole.id, "tag": f"hole_{i}", "type": "Hole"})
         
    return features
```

### The "Selector" Tool

When the user says "Chamfer the top edge", the agent needs to select it in `build123d`.
Instead of hardcoding IDs (which change!), we inject **Semantic Selectors**:

<!-- FIXME: the former should never occur in the first place. -->
```python
# OLD (Fragile)
edges = part.edges()[14] 


# NEW (Robust)
edges = part.edges().filter_by(Axis.Z).sort_by(Axis.Z)[-1] # Top-most edges
```

The Adaptability Agent is trained to write *Selectors*, not hard indices.

### Frontend Integration (Human-in-the-loop)

We will need a dedicated CAD viewer, e.g. Yet Another Cad Viewer, which allows to select faces, groups of faces, etc.

It would send the face IDs or other metadata to the backend as augmented fro the propmt. Alternatively, it would send a number of images to the LLM to choose
<!-- TODO: I don't know hwich -->

## Visualization of Changes (Diffing)

How do we show the user what changed?

### 1. Geometric Diff (Boolean)

We perform a boolean operation between Old and New models.

* `Added Material = New - Old` (Color: Green)
* `Removed Material = Old - New` (Color: Red)
* `Unchanged = Old & New` (Color: Grey/Transparent)

### 2. Code Diff

Standard Git-style diff of the generated Python script.

### 3. Metric Diff

* "Mass: 50g -> 55g (+10%)"
* "Cost: $2.00 -> $2.10 (+5%)"
<!-- good idea. -->

## Tech Stack & Reproducibility

* **Build123d**: Specifically its robust **Selector API** (`faces().sort_by()`, `edges().filter_by()`). This is the backbone of robust editing.
* **Git (Reproducibility)**: implementation of "Undo/Redo" is **Git**.
  * Every "Edit Step" creates a commit in a bare repo at `/var/data/sessions/{id}.git`.
  * To "Undo", we `git reset --hard HEAD~1`.
  * To "Reproduce", we can replay the git log on a fresh container.
* **Three.js**: For the frontend viewer with raycasting support.
* **Difftastic**: For syntax-aware code diffing (if we want to show nice diffs to the user).
* **WebSockets**: For real-time updates during the edit session.

## Failure Modes & Recovery

* **"Topological Naming Problem"**: A classic CAD issue. If the user deletes a face, subsequent operations referencing that face will fail.
  * *Strategy*: The Agent must handle `SelectorNotFoundError`. If a selector returns empty, the Agent must pause and ask the user (or re-analyze the new topology).
* **Invalid Geometry**: The edit creates a self-intersecting model.
  * *Strategy*: `validate_and_price` is run after *every* edit. If validation fails, the agent auto-reverts and tries a different approach (or informs the user).
