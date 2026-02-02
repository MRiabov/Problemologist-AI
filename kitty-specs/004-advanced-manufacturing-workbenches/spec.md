# Feature Specification: Advanced Manufacturing Workbenches and DFM Analysis

**Feature**: 004-advanced-manufacturing-workbenches
**Status**: Draft
**Mission**: software-dev

## 1. Overview

This feature expands the **Problemologist-AI** environment beyond simple 3D printing by implementing two advanced manufacturing workbenches: **CNC Milling** and **Injection Molding (IM)**.

These processes introduce strict **Design for Manufacturing (DFM)** constraints—such as draft angles, tool access, and wall thickness—that the agent must navigate. Unlike 3D printing, where "if it meshes, it prints," these technologies require the agent to fundamentally alter geometry to suit the production method.

Additionally, this feature introduces a **Manufacturability Analysis Tool** (`check_manufacturability`) that provides the agent with detailed DFM reports and cost estimates, enabling it to make trade-offs between "Production Volume" (1 vs 10,000 parts) and "Unit Cost."

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Implement CNC Workbench**: strict checks for 3-axis machinability (tool access, corner radii, undercuts).
2. **Implement Injection Molding Workbench**: strict checks for moldability (draft angles, wall thickness, simple pull/no-undercuts).
3. **Complex Cost Modeling**: Implement volume-dependent pricing (High Setup/Low Unit Cost for IM vs. Low Setup/High Unit Cost for CNC).
4. **DFM Feedback Loop**: Provide a tool that gives the agent granular feedback (e.g., "Face A has 0° draft, requires 2°") so it can iterate on the design.

### 2.2. Success Criteria

* **CNC Validation**: Correctly identifies and rejects parts with:
  * Internal vertical corners sharper than the specified tool radius.
  * Undercuts not accessible from the Z-axis (for 3-axis assumption).
* **IM Validation**: Correctly identifies and rejects parts with:
  * Vertical faces having $< 2^\circ$ draft angle relative to the pull direction.
  * Wall thickness variance $> 30\%$.
  * Undercuts preventing mold separation.
* **Cost Sensitivity**: The system returns significantly different optimal choices for "1 part" (CNC/Print) vs "10,000 parts" (Injection Molding).
* **Agent Tool**: `check_manufacturability` returns a structured JSON report valid for agent consumption within 5 seconds for standard test parts.

## 3. User Stories

* **As an Agent**, I want to check if my design can be CNC milled so that I don't submit physically impossible geometries.
* **As an Agent**, I want to know specifically *which* faces fail the draft angle check so I can apply the `draft()` operation to them.
* **As an Agent**, I want to compare the cost of making 1,000 units via CNC vs. Injection Molding so I can choose the most economical process.
* **As an Agent**, I want to receive feedback on "sharp internal corners" so I can add fillets compatible with a milling cutter.

## 4. Functional Requirements



### 4.1. CNC Milling Validation



*   **System Integration**: The validation logic must integrate with the existing Workbench system.

*   **Geometric Constraints**:

    1.  **Tool Accessibility**: The system shall detect internal corners with a radius smaller than the configurable minimum tool radius (default: 3mm).

    2.  **3-Axis Visibility**: The system shall verify that all machined surfaces are accessible from a defined set of approach vectors (default: +Z axis only for simple setup).

    3.  **Undercut Detection**: The system shall reject geometry that is occluded from the tool path (i.e., material overhangs that a rigid tool cannot reach).

*   **Cost Model**:

    *   **Setup Cost**: Calculate based on the number of unique orientations required to machine the part.

    *   **Run Cost**: Calculate based on material volume removal rate and machine hourly rate.

    *   **Material Cost**: Calculate based on bounding box volume (stock size) and material density/price.



### 4.2. Injection Molding Validation



*   **System Integration**: The validation logic must integrate with the existing Workbench system.

*   **Geometric Constraints**:

    1.  **Draft Angle Analysis**: The system shall verify that all faces parallel to the draw direction (mold pull axis) have a draft angle greater than or equal to the minimum threshold (default: $2^\circ$).

    2.  **Undercut Detection**: The system shall ensure the part can be separated from a simple 2-part mold (no trapped geometry preventing ejection).

    3.  **Wall Thickness Analysis**: The system shall sample the geometry to ensure wall thickness stays within a specified `[min, max]` range and does not vary by more than a set threshold (to prevent defects).

*   **Cost Model**:

    *   **Tooling Cost**: Apply a high fixed cost derived from the part's surface area and geometric complexity.

    *   **Unit Cost**: Apply a low per-unit cost based on actual part volume and material price.



### 4.3. Manufacturability Analysis Tool



The environment shall expose a new tool to the agent.



*   **Functionality**: Accepts a design file, a target process (CNC or Injection Molding), and a production quantity.

*   **Output**: A structured report containing:

    *   **Manufacturability Status**: Boolean indicating if the part passes all DFM checks.

    *   **Cost Estimate**: Per-unit and total cost for the specified quantity.

    *   **Violation Report**: A list of specific DFM violations, including the type of violation (e.g., "No Draft", "Undercut"), location, and a descriptive message.

*   **Visualization** (Optional): A visual overlay highlighting problematic areas (e.g., red faces for missing draft).



## 5. Technical Design



### 5.1. Geometric Analysis Implementation



*   **Core Library**: `build123d` (wrapping OpenCASCADE).

*   **Workbench Classes**: Implement `CNCWorkbench` and `InjectionMoldingWorkbench` inheriting from `src.workbenches.base.Workbench`.

*   **Draft Analysis**:

    *   Iterate over topological faces (`part.faces()`).

    *   Compute normal vector $\vec{n}$ at face center.

    *   Check angle against Pull Vector $\vec{p}$ (usually $+Z$).

    *   Formula: $|\arccos(\vec{n} \cdot \vec{p}) - 90^\circ| \ge \theta_{min}$.

*   **Undercut/Raycasting**:

    *   Use raycasting grids (via `trimesh` or OCP `BRepExtrema`) to check for self-occlusion from the tool/pull direction.

*   **Wall Thickness**:

    *   Raycast internal normals or use Sphere-fitting (Medial Axis Transform) approximation if feasible, otherwise simple cross-section sampling.



### 5.2. Integration



*   **Files**: Create `src/workbenches/cnc.py` and `src/workbenches/injection_molding.py`.

*   **Tool Wrapper**: Update `src/environment/tools.py` to expose the `check_manufacturability` function, which delegates to the appropriate Workbench class.

## 6. Assumptions & Constraints

* **Simplification**: We assume a "Simple 2-Part Mold" for Injection Molding (no side actions/lifters) to keep the reasoning task tractable for now.
* **Simplification**: We assume "3-Axis Machining" for CNC. 5-axis is out of scope.
* **Performance**: Detailed DFM checks (especially raycasting) can be slow. Implementation should use coarse meshes or bounding-box approximations where possible to keep runtime < 5s.
* **Material**: Default to "Aluminum 6061" for CNC and "ABS" for Injection Molding for pricing constants.

## 7. Economic Optimization & Record System

To drive the agent towards highly efficient designs, the environment supports an economic "Record" system.

### 7.1. Cost Targets
Tasks can be defined with explicit economic constraints:
* `target_quantity`: The production volume (e.g., 10,000 units).
* `max_unit_cost`: The maximum allowable per-unit cost.

### 7.2. The Record System
The system tracks the lowest unit cost achieved for any given benchmark scenario.
* **Objective**: When a record exists, the agent's primary objective (beyond functional success) is to produce a design with a `unit_cost < current_record`.
* **RL Rollouts**: This mechanism allows for the generation of "Optimization Rollouts," where the agent iteratively refines a design to reduce material volume, simplify machining orientations, or optimize for injection molding wall thickness to beat previous records.
