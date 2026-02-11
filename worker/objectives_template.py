from __future__ import annotations

from pathlib import Path

OBJECTIVES_YAML_TEMPLATE = """# [TEMPLATE] - DO NOT USE FOR SIMULATION
# =============================================================================
# OBJECTIVES.YAML - Your Task Definition
# =============================================================================
# This file defines WHAT you must achieve. Read it carefully before planning.
#
# YOUR MISSION: Guide the `moved_object` into the `goal_zone` while:
#   1. Staying WITHIN the `build_zone` (you cannot build outside it)
#   2. AVOIDING all `forbid_zones` (contact = failure)
#   3. Respecting `max_unit_cost` and `max_weight` constraints
#
# The environment and moving_parts are READ-ONLY. You design parts that
# interact with them, not modify them.
# =============================================================================

objectives:
  # SUCCESS: The moved_object's center enters this volume
  goal_zone:
    min: [x_min, y_min, z_min]
    max: [x_max, y_max, z_max]

  # FAILURE: Any contact with these volumes fails the simulation
  forbid_zones:
    - name: "obstacle_collision_zone"
      min: [x1, y1, z1]
      max: [x2, y2, z2]
    # Additional forbid zones may be listed here

  # CONSTRAINT: Your entire design MUST fit within these bounds
  # Parts placed outside will fail validation
  build_zone:
    min: [x, y, z]
    max: [x, y, z]

# Hard simulation boundaries - objects leaving this volume = failure
simulation_bounds:
  min: [-50, -50, 0]
  max: [50, 50, 100]

# -----------------------------------------------------------------------------
# THE OBJECT YOU MUST DELIVER
# -----------------------------------------------------------------------------
# This object spawns at `start_position` (with runtime jitter applied).
# Your design must reliably guide it to the goal_zone.
moved_object:
  label: "projectile_ball"
  shape: "sphere"
  # Static randomization: shape varies between benchmark runs
  static_randomization:
    radius: [5, 10]  # [min, max] - actual value chosen per benchmark variant
  start_position: [x, y, z]
  # Runtime jitter: small position variation per simulation run
  # Your solution must handle ALL positions within this range
  runtime_jitter: [2, 2, 1]  # [+/-x, +/-y, +/-z] mm

# -----------------------------------------------------------------------------
# ENVIRONMENT MOVING PARTS (READ-ONLY)
# -----------------------------------------------------------------------------
# These exist in the environment. You CANNOT modify them, but you CAN
# design parts that interact with them (e.g., attach to motor shafts,
# use passive sliders as triggers).
moving_parts:
  - name: "feeder_motor"
    type: "motor"
    position: [x, y, z]
    dof: "rotate_z"  # Degrees of freedom: rotate_x/y/z, slide_x/y/z
    control:
      mode: "sinusoidal"  # Options: constant, sinusoidal, on_off
      speed: 1.0          # rad/s (for rotate) or units/s (for slide)
      frequency: 0.5      # Hz - for sinusoidal mode
    description: "Rotates clockwise to push objects"

  - name: "passive_slider"
    type: "passive"  # Moves only when external force applied
    position: [x, y, z]
    dof: "slide_y"
    description: "Slides freely along Y when impacted"

# -----------------------------------------------------------------------------
# YOUR CONSTRAINTS
# -----------------------------------------------------------------------------
# These are challenging but achievable. Exceeding them = rejection.
constraints:
  max_unit_cost: 50.00  # USD - total cost of your manufactured parts
  max_weight: 1.2       # kg - total weight of your design

# Randomization metadata (for reproducibility)
randomization:
  static_variation_id: "v1.2"  # Which static variant this is
  runtime_jitter_enabled: true
"""


def ensure_objectives_yaml(root: Path) -> None:
    """Create objectives.yaml template in the session root if missing."""
    path = root / "objectives.yaml"
    if path.exists():
        return
    path.write_text(OBJECTIVES_YAML_TEMPLATE, encoding="utf-8")
