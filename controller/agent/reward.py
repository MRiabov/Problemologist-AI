from pathlib import Path

import yaml
from pydantic import BaseModel


class MilestoneConfig(BaseModel):
    """Configuration for a single reward milestone."""

    weight: float
    description: str
    partial: bool = False

    # Optional formulas for different evaluation types
    formula: str | None = None  # Generic formula
    penalty_formula: str | None = None  # Formula for calculating continuous penalty
    success_formula: str | None = None  # Formula for success case
    failure_formula: str | None = None  # Formula for failure case

    # Result mapping
    minimum_score: float | None = None
    binary: bool = False  # If True, only 0.0 or 1.0 (weighted by 'weight')

    # Feedback strings for GEPA optimization
    success_feedback: str | None = None
    failure_feedback: str | None = None


class AgentRewardConfig(BaseModel):
    """Configuration for an agent's reward structure."""

    description: str
    milestones: dict[str, MilestoneConfig]


class RewardConfig(BaseModel):
    """Top-level reward configuration for DSPy optimization."""

    bootstrap_threshold: float
    benchmark_generator: dict[str, AgentRewardConfig]
    engineer: dict[str, AgentRewardConfig]
    shared: dict[str, AgentRewardConfig]


def load_reward_config(config_path: Path | None = None) -> RewardConfig:
    """
    Loads and validates the reward configuration from a YAML file.

    Args:
        config_path: Path to the reward_config.yaml file.
                     Defaults to config/reward_config.yaml relative to project root.

    Returns:
        RewardConfig: Validated pydantic model.
    """
    if config_path is None:
        # Assuming we are running from project root or similar structure
        config_path = Path("config/reward_config.yaml")

    if not config_path.exists():
        # Fallback to absolute path search if relative fails in some contexts
        # (e.g. running from a subdirectory)
        root_path = Path(__file__).parent.parent.parent
        config_path = root_path / "config" / "reward_config.yaml"

    with config_path.open() as f:
        config_data = yaml.safe_load(f)

    return RewardConfig(**config_data)
