# Phase 1: Data Models - Electromechanical Integration (WP3)

## 1. Electronics Schema Extensions

Refining `shared/models/schemas.py` to support 3D routing and detailed components.

```python
from pydantic import BaseModel, Field
from typing import List, Tuple, Optional

class TerminalPoint(BaseModel):
    component_id: str
    terminal_name: str  # e.g., "+", "-", "vcc", "gnd"
    pos: Optional[Tuple[float, float, float]] = None # Relative to comp origin

class WireSegment(BaseModel):
    wire_id: str
    from_terminal: TerminalPoint
    to_terminal: TerminalPoint
    waypoints: List[Tuple[float, float, float]] = [] # Global coordinates
    gauge_awg: int = 18
    color: str = "red"
    routed_in_3d: bool = False

class ElectronicsSection(BaseModel):
    power_supply: PowerSupplyConfig
    components: List[ComponentConfig]
    wiring: List[WireSegment]
```

## 2. Electrical Agent Coordination Map

The "Visual Map" structure provided to the Electrical Engineer sub-agent.

```python
class AttachmentSite(BaseModel):
    site_id: str
    parent_part: str
    pos: Tuple[float, float, float] # Global
    normal: Tuple[float, float, float] # Surface normal for orientation
    available_terminals: List[str]

class ObstacleVolume(BaseModel):
    part_id: str
    bbox_min: Tuple[float, float, float]
    bbox_max: Tuple[float, float, float]
    is_conductive: bool = False

class ElectricalCoordinationMap(BaseModel):
    attachment_sites: List[AttachmentSite]
    obstacles: List[ObstacleVolume]
    current_wiring: List[WireSegment]
```

## 3. Spline Query Tool Interface

Interface for the tool used by the agent to validate proposed routes.

```python
class SplineQueryRequest(BaseModel):
    waypoints: List[Tuple[float, float, float]]
    wire_diameter_mm: float
    required_clearance_mm: float = 2.0

class SplineQueryResponse(BaseModel):
    is_valid: bool
    total_length_mm: float
    intersections: List[str] = [] # List of part_ids collided with
    min_distance_mm: float
    violations: List[str] = []
```
