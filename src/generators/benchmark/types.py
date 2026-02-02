from typing import TypedDict, List, Optional, Dict, Any

class ScenarioAssets(TypedDict):
    mjcf: str
    meshes: List[str]
    images: List[str]

class ScenarioRandomization(TypedDict):
    seed_range: List[int]
    parameters: List[str]
    scale: List[float] # [sx, sy, sz]

class ScenarioValidation(TypedDict):
    passed: bool
    max_velocity: float

class ScenarioManifest(TypedDict):
    id: str
    tier: str  # "spatial" or "kinematic"
    description: str
    script_path: str
    assets: ScenarioAssets
    randomization: ScenarioRandomization
    validation: ScenarioValidation

class ValidationReport(TypedDict):
    is_valid: bool
    error_message: Optional[str]
    sim_duration: float
    max_energy: float
