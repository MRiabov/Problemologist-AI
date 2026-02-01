# Data Model: Advanced Manufacturing Workbenches

## Entities

### 1. Manufacturing Report
The primary output of the `check_manufacturability` tool.

| Field | Type | Description |
|---|---|---|
| `status` | string | "pass" or "fail" |
| `process` | string | "cnc_milling" or "injection_molding" |
| `manufacturability_score` | float | 0.0 to 1.0 (1.0 = perfect) |
| `violations` | List[Violation] | List of specific DFM issues detected. |
| `cost_analysis` | CostAnalysis | Detailed breakdown of estimated costs. |
| `visualization_data` | VisualizationData | (Optional) Data for UI overlay. |

### 2. Violation
Represents a specific geometric feature that fails DFM checks.

| Field | Type | Description |
|---|---|---|
| `type` | string | e.g., "undercut", "draft_angle", "thin_wall" |
| `description` | string | Human-readable explanation. |
| `severity` | string | "critical" (impossible) or "warning" (expensive) |
| `affected_faces` | List[int] | Indices of mesh faces causing the violation. |
| `location` | [x, y, z] | Approximate centroid of the violation. |

### 3. Cost Analysis
Breakdown of the manufacturing cost.

| Field | Type | Description |
|---|---|---|
| `quantity` | int | Production volume requested. |
| `unit_cost` | float | Cost per single part. |
| `total_cost` | float | `unit_cost * quantity + tooling_cost` |
| `setup_cost` | float | Fixed NRE (Non-Recurring Engineering) or tooling cost. |
| `material_cost` | float | Cost of raw material per part. |
| `processing_cost` | float | Machine time cost per part. |

### 4. Visualization Data
Lightweight data for rendering feedback.

| Field | Type | Description |
|---|---|---|
| `highlight_faces` | List[int] | Face indices to colorize (e.g., red). |
| `vectors` | List[Vector] | Optional debug vectors (e.g., surface normals). |
