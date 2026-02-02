from typing import TypedDict


class ScenarioAssets(TypedDict):
    mjcf: str
    meshes: list[str]
    images: list[str]


class ScenarioRandomization(TypedDict):
    seed_range: list[int]
    parameters: list[str]
    scale: list[float]  # [sx, sy, sz]


class ScenarioValidation(TypedDict):
    passed: bool
    max_velocity: float


class ScenarioManifest(TypedDict):
    id: str
    tier: str  # "spatial" or "kinematic"
    description: str
    script_path: str
    target_quantity: int
    max_unit_cost: float
    cost_record: float | None  # The lowest unit cost achieved so far
    assets: ScenarioAssets
    randomization: ScenarioRandomization
    validation: ScenarioValidation


class ValidationReport(TypedDict):
    is_valid: bool
    error_message: str | None
    sim_duration: float
    max_energy: float
