# Research: Advanced Manufacturing Workbenches

## 1. Geometric Analysis with Trimesh

To perform DFM checks efficiently, we will convert `build123d` solid models into `trimesh` mesh objects.

### 1.1 Conversion Strategy
`build123d` does not directly output `trimesh` objects, but it can export STL to a buffer.
```python
import trimesh
from build123d import Part, export_stl
import io

def part_to_trimesh(part: Part) -> trimesh.Trimesh:
    with io.BytesIO() as f:
        export_stl(part, f)
        f.seek(0)
        mesh = trimesh.load(f, file_type='stl')
    return mesh
```

### 1.2 Draft Angle Analysis (Injection Molding)
*   **Goal**: Ensure faces parallel to pull direction have sufficient slope.
*   **Method**:
    1.  Get face normals: `mesh.face_normals`
    2.  Define Pull Vector: `P = [0, 0, 1]`
    3.  Compute angle: $\theta = \arccos(\vec{n} \cdot \vec{P})$
    4.  **Check**:
        *   Faces with $\theta \approx 90^\circ$ (within tolerance) are vertical walls requiring draft.
        *   Fail if $|90^\circ - \theta| < \text{min\_draft}$.

### 1.3 Undercut Detection (CNC & IM)
*   **Goal**: Detect geometry occluded from the tool/mold.
*   **Method (Raycasting)**:
    1.  Target: Center points of all faces (`mesh.triangles_center`).
    2.  Source: Target points + (Pull Vector * epsilon).
    3.  Direction: Pull Vector (for IM) or Tool Vector (for CNC).
    4.  Cast rays: `mesh.ray.intersects_any(origins, vectors)`
    5.  **Logic**: If a ray hitting a face center intersects *other* geometry on its way out, that face is undercut (trapped).

## 2. Configuration Schema (`manufacturing_config.yaml`)

We will use a central YAML file to control costs and constraints.

```yaml
defaults:
  currency: "USD"

cnc:
  materials:
    aluminum_6061:
      density_g_cm3: 2.7
      cost_per_kg: 5.50
      machine_hourly_rate: 80.00
  constraints:
    min_tool_radius_mm: 3.0
    default_axis: "Z"

injection_molding:
  materials:
    abs:
      density_g_cm3: 1.04
      cost_per_kg: 2.50
  constraints:
    min_draft_angle_deg: 2.0
    min_wall_thickness_mm: 1.0
    max_wall_thickness_mm: 4.0
  costs:
    base_mold_cost: 5000.00
    mold_cost_per_surface_area_cm2: 0.50
```

## 3. Caching Strategy

DFM checks are expensive. We will cache results.

*   **Key**: `hash(geometry) + hash(process_params)`
*   **Implementation**:
    *   `trimesh` provides a fast CRC/hash of the mesh data: `mesh.identifier` (MD5 of geometry).
    *   We will use a simple in-memory `functools.lru_cache` wrapper around the analysis functions, passing the mesh identifier string instead of the full object.

## 4. Dependencies
*   **Runtime**: `trimesh[easy]` (includes `scipy`/`numpy` helpers), `pyyaml`.
*   **Dev**: `pytest`.

```