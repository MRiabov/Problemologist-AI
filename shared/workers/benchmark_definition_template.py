from __future__ import annotations

from pathlib import Path

BENCHMARK_DEFINITION_YAML_TEMPLATE = """# [TEMPLATE] - DO NOT USE FOR SIMULATION
# =============================================================================
# BENCHMARK_DEFINITION.YAML - Your Task Definition
# =============================================================================
# This file defines WHAT you must achieve and the benchmark-owned fixture
# metadata you may rely on before planning.
#
# YOUR MISSION: Guide the `moved_object` into the `goal_zone` while:
#   1. Staying WITHIN the `build_zone` (you cannot build outside it)
#   2. AVOIDING all `forbid_zones` (contact = failure)
#   3. Respecting cost and weight constraints
#
# Benchmark-owned environment geometry and metadata in this file are READ-ONLY.
# Engineering assembly motion metadata is stored under engineering
# assembly_definition.yaml final_assembly.parts and is also READ-ONLY once
# written.
# =============================================================================

objectives:
  # SUCCESS: The moved_object's center enters this volume
  goal_zone:
    min: [6.0, -2.0, 0.0]
    max: [10.0, 2.0, 4.0]

  # FAILURE: Any contact with these volumes fails the simulation
  forbid_zones:
    - name: "obstacle_collision_zone"
      min: [3.0, -3.0, 0.0]
      max: [4.0, 3.0, 4.0]
    # Additional forbid zones may be listed here

  # CONSTRAINT: Your entire design MUST fit within these bounds
  # Parts placed outside will fail validation
  build_zone:
    min: [-10.0, -10.0, -10.0]
    max: [10.0, 10.0, 10.0]

benchmark_parts:
  - part_id: "environment_fixture"
    label: "environment_fixture"
    metadata:
      fixed: true
      material_id: "aluminum_6061"
  # Every benchmark part must have a unique `part_id` and a unique `label`.
  # Authored top-level build123d part labels must also be unique and must not
  # use the reserved `environment` label or the reserved `zone_` namespace,
  # which are owned by the scene root and simulator-generated objective bodies.

# Hard simulation boundaries - objects leaving this volume = failure
simulation_bounds:
  min: [-50.0, -50.0, 0.0]
  max: [50.0, 50.0, 100.0]

# -----------------------------------------------------------------------------
# THE OBJECT YOU MUST DELIVER
# -----------------------------------------------------------------------------
# This object spawns at `start_position` (with runtime jitter applied).
# Your design must reliably guide it to the goal_zone.
# The full runtime AABB (`start_position +/- runtime_jitter +/- max(radius)`)
# must remain inside `build_zone` on every axis.
moved_object:
  label: "projectile_ball"
  shape: "sphere"
  material_id: "abs"
  # Static randomization: shape varies between benchmark runs
  static_randomization:
    radius: [1.0, 2.0]  # [min, max] - actual value chosen per benchmark variant
  start_position: [0.0, 0.0, 0.0]
  # Runtime jitter: small position variation per simulation run
  # Your solution must handle ALL positions within this range
  runtime_jitter: [0.5, 0.5, 0.5]  # [+/-x, +/-y, +/-z] mm

# -----------------------------------------------------------------------------
# YOUR CONSTRAINTS
# -----------------------------------------------------------------------------
# Planner/runtime contract:
# - Planner authors `estimated_solution_*` fields.
# - Runtime derives permissive max caps (1.5x estimates) for engineering gates.
constraints:
  estimated_solution_cost_usd: 33.33
  estimated_solution_weight_g: 800.0

# Randomization metadata (for reproducibility)
randomization:
  static_variation_id: "v1.2"  # Which static variant this is
  runtime_jitter_enabled: true
"""


def ensure_benchmark_definition_yaml(root: Path) -> None:
    """Create benchmark_definition.yaml template in the session root if missing."""
    path = root / "benchmark_definition.yaml"
    if path.exists():
        return
    path.write_text(BENCHMARK_DEFINITION_YAML_TEMPLATE, encoding="utf-8")
