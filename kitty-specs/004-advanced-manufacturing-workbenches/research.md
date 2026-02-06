# Research: Advanced Manufacturing Workbenches

## Deterministic Manufacturability Checks

To ensure the agents produce realistic hardware, we implement a set of deterministic "Workbenches" that judge geometry based on physical constraints.

### 1. 3D Printing (FDM/SLS)

- **Constraint**: Minimum wall thickness and overhang angles.
- **Algorithm**: Ray-casting to find thin sections and calculating face normals relative to the build plate.
- **Cost**: Based on volume + material prep time.

### 2. CNC Milling (3-axis)

- **Constraint**: Tool accessibility and internal radii.
- **Algorithm**: Visibility check (ray-cast from the spindle axis) to detect undercuts. Verification that all internal corners satisfy `min_tool_radius`.
- **Cost**: Material block cost + machine hourly rate based on volume removed and tool changes.

### 3. Injection Molding

- **Constraint**: Draft angles and uniform wall thickness.
- **Algorithm**: Normal-angle check against the pull direction. Fail if draft is below `min_draft_angle`.
- **Cost**: High initial mold cost (calculated by surface area and complexity) divided by target quantity.

## Cost Modeling Strategy

**Decision**: Use a centralized `manufacturing_config.yaml`.
**Rationale**:

- Provides a "ground truth" for materials, electricity costs, and labor rates.
- Allows the benchmark generator to simulate different economic conditions (e.g., "high-cost aluminum scenario").

## Feedback Loop: The DFM Report

**Decision**: Feedback via Markdown reports.
**Rationale**:

- When a `validate_and_price()` call fails, the agent receives a Markdown report detailing the exact violation (e.g., "Undercut detected at [X,Y,Z]").
- Score is calculated from 0.0 to 1.0; agents can be prompted to optimize for a specific score.

## Distributed Logic

**Decision**: Benchmarks run on the **Worker** for speed.
**Rationale**:

- Complex ray-casting (`trimesh`) over high-res meshes is compute-intensive.
- Keeping it on the worker avoids shipping large STL files over the network.
