# Mathematics of Fastener Insertion

This document details the geometric logic used to position fasteners relative to holes, specifically derived from `recalculate_fastener_pos_with_offset_to_hole` in `repairs_components/geometry/fasteners.py`.

## Core Concept

The system calculates the **Constraint Position** ($P_{fastener}$) based on the **Hole Position** ($P_{hole}$) and **Hole Orientation** ($Q_{hole}$), while accounting for the fastener length ($L$) and hole type.

The fundamental transformation is:
$$ P_{fastener} = P_{hole} + (Q_{hole} \times Offset_{vec}) $$

Where $Offset_{vec}$ is a translation along the hole's Z-axis (local `(0, 0, z)`).

## Three Insertion Cases

### Case 1: Through Hole (Standard)

*Connecting the head to the surface.*

- **Scenario**: A fastener is inserted into a through-hole (where `depth` is `None`).
- **Logic**: The fastener head ($Joint_A$) aligns directly with the hole entrance.
- **Offset**: $0$
- **Result**:
  $$ P_{fastener} = P_{hole} $$

### Case 2: Blind Hole (Bottom constraint)

*Connecting the tip to the bottom of a hole.*

- **Scenario**: A fastener is fully inserted into a blind hole (e.g., creating a threaded connection at the bottom).
- **Goal**: The tip of the fastener ($Z = -L$) must touch the bottom of the hole ($Z = -D_{hole}$).
- **Offset Calculation**:
  To align the tip at $-L$ with hole bottom at $-D_{hole}$, we must shift the fastener up/down by the difference.
  $$ Offset_Z = L - D_{hole} $$
- **Why?**: The fastener origin (Head) needs to be at $Z = L - D_{hole}$ relative to the hole entrance for the tip to be at $-D_{hole}$.
  *(Note: This assumes the fastener is longer than the hole depth, leaving the head PROTRUDING. If $L == D_{hole}$, offset is 0.)*

### Case 3: Partial Insertion (Stacked Parts)

*Connecting a fastener that is ALREADY traversing a top part.*

- **Scenario**: Fastener passes through Part A (thickness $T_{top}$) and enters Part B. We are calculating the constraint for **Part B**.
- **Goal**: The fastener is already "consumed" by $T_{top}$ length. The constraint to Part B should be relative to where the fastener enters Part B.
- **Offset**:
  $$ Offset_Z = L - T_{top} $$
- **Logic**: effectively treating the connection point as if the fastener were $L - T_{top}$ long starting from Part B's surface.

## Quaternion Alignment

The orientation is strictly copied from the hole to the fastener to ensure coaxial alignment.
$$ Q_{fastener} = Q_{hole} $$

## Summary Table

| Case | Condition | Offset (Z-axis) | Resulting Position |
| :--- | :--- | :--- | :--- |
| **Through Hole** | `is_through == True` | $0$ | $P_{hole}$ |
| **Blind Hole** | `is_through == False` | $L - D_{hole}$ | $P_{hole} + Q \times (0, 0, L - D_{hole})$ |
| **Partial** | `top_depth > 0` | $L - T_{top}$ | $P_{hole} + Q \times (0, 0, L - T_{top})$ |

*Variables:*

- $L$: Fastener Length
- $D_{hole}$: Blind Hole Depth
- $T_{top}$: Thickness of the top part (depth of the first hole it passed through)
