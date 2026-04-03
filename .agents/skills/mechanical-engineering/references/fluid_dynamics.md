# Fluid Dynamics (MPM) Principles

## Core Concepts

### 1. Material Point Method (MPM)

Used for simulating fluid-structure interaction. Fluids are represented as particles that carry mass and momentum.

### 2. Fluid Properties

- **Viscosity:** Resistance to flow (e.g., Water ~1.0, Honey ~10,000).
- **Density:** Mass per unit volume ($kg/m^3$).
- **Surface Tension:** Force that causes the surface of a liquid to behave like a stretched elastic sheet.

## Objectives and Metrics

### 1. Fluid Containment

- **Goal:** Keep fluid particles within a specified zone (e.g., a cup or tank).
- **Metric:** Containment Ratio ($R = \\frac{N\_{inside}}{N\_{total}}$).
- **Threshold:** Typically > 0.95 (95% containment).

### 2. Flow Rate

- **Goal:** Measure or control the rate of fluid passing through a gate.
- **Metric:** Particles per second crossing a plane.
- **Threshold:** Specified in the benchmark (e.g., > 100 particles/sec).

## Simulation Constraints

- **Backend:** Fluid simulation REQUIRES the `genesis` physics backend.
- **Compute:** Highly GPU intensive.
- **Smoke Test Mode:** Capped to 5,000 particles for rapid iteration.
- **Failure Reason:** `FLUID_OBJECTIVE_FAILED` if metrics are not met.
