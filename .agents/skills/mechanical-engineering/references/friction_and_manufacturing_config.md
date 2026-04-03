# Friction And Manufacturing Config

## Contents

1. Source of truth
2. First-order sliding rule
3. Current workspace coefficients
4. How to use the numbers

## Source Of Truth

Read the workspace `manufacturing_config.yaml` first. If a legacy workspace only has `config/manufacturing_config.yaml`, use that file instead.

Use the file that actually exists in the workspace. Do not invent a coefficient.

## First-Order Sliding Rule

For a passive slide, ramp, or chute:

`mu_static < tan(theta)`

Equivalently:

`theta > atan(mu_static)`

Equality is not enough. Use a safety margin above the threshold if the design must survive jitter, bounce, or slight placement error.

## Current Workspace Coefficients

From the seeded `manufacturing_config.yaml`:

| Material | friction_coef |
| -- | -: |
| `aluminum_6061` | `0.61` |
| `steel_carbon` | `0.60` |
| `steel_structural` | `0.62` |
| `abs` | `0.50` |
| `hdpe` | `0.22` |
| `hardwood` | `0.50` |
| `silicone_rubber` | `0.80` |
| `rubber_generic` | `0.90` |

Process-specific material blocks in the same file:

| Block | Material | friction_coef |
| -- | -- | -: |
| `cnc` | `aluminum_6061` | `0.60` |
| `injection_molding` | `abs` | `0.50` |
| `three_dp` | `abs` | `0.40` |

## How To Use The Numbers

- Use the coefficient for the actual contacted surface when checking whether gravity can start motion.
- For passive transfer, friction matters before capture width does.
- If `tan(theta)` is below the relevant coefficient, the object may settle instead of moving.
- If the mechanism stalls in simulation, change slope, surface, or mechanism family before adding more geometry.
- The helper script `scripts/get_mechanical_properties.py` can dump the current workspace material properties.
