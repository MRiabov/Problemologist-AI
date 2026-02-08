# Benchmark Scenario Plan

## Learning Objective
<!-- Describe what the agent needs to learn or demonstrate. -->
- **Goal**: [e.g., Test spatial reasoning, gravity manipulation, etc.]
- **Key Concepts**: [e.g., Slopes, funnels, containment]

## Geometry

### Static Environment
<!-- Fixed parts of the scene. Coordinates are relative to world origin (0,0,0). -->
- **Ground/Base**: [Description and approximate dimensions/location]
- **Walls/Obstacles**: [Description and approximate dimensions/location]

### Moving Parts (Active)
<!-- Motors, actuators, or parts that move under their own power. -->
- **Motor 1**: [Type, Location, Function]

### Input Object
<!-- The object to be manipulated (e.g., the ball). -->
- **Shape**: [e.g., Sphere, Cube, Custom]
- **Dimensions**: [e.g., Radius=5mm]
- **Spawn Position**: [X, Y, Z]
- **Randomization**:
  - **Position**: [e.g., X ± 5mm, Y ± 5mm]
  - **Properties**: [e.g., Mass, Friction]

## Objectives

### Goal Zone
<!-- Where the input object must end up. -->
- **Type**: AABB (Axis-Aligned Bounding Box)
- **Location**: [X, Y, Z]
- **Dimensions**: [Width, Depth, Height]

### Forbid Zones
<!-- Areas the input object or agent parts must NOT touch. -->
- **Zone 1**: [Location and Dimensions]

## Constraints

- **Max Volume**: [e.g., 100x100x100 mm]
- **Material**: [e.g., PLA, Aluminum]

## Randomization Strategy
<!-- Summary of how the scene varies between runs. -->
- [Strategy 1]
- [Strategy 2]
