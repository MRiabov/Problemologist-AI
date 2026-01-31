# Problemologist Environment Scope

The Problemologist environment is a high-fidelity Reinforcement Learning (RL) environment designed to automate mechanical engineering design and optimization tasks. It integrates parametric CAD modeling with physical simulation feedback, allowing an RL agent (such as DreamerV3-Flax) to act as an autonomous engineer.

## 1. Core Objective

The goal of the agent within this environment is to design a mechanical assembly that satisfies a set of geometric objectives while staying within strict physical and economic constraints (weight, cost, size, manufacturability).

## 2. System Architecture

The environment follows a design-check-simulate-reward cycle:

1. **Design**: The agent selects parametric modeling actions (e.g., "Create a box at (x,y,z) with dimensions (l,w,h)").
2. **Check**: The environment performs instantaneous manufacturability and constraint checks (e.g., "Is this part too thin?", "Does it exceed the budget?").
3. **Simulate**: If the design is valid, it is exported (STEP format) and passed to external solvers (FEM, CFD, etc.) for performance evaluation.
4. **Reward**: The agent receives feedback based on objective completion and penalty terms.

## 3. Workbenches (Domain-Specific Contexts)

The environment is organized into "Workbenches," each representing a different manufacturing or assembly process:

### A. Injection Molding Workbench

Focuses on designing plastic parts intended for injection molding.

* **Manufacturability Constraints**:
  * **Thickness Check**: Ensures parts are within a specific range (default ~5mm) to avoid cooling issues or structural weakness.
  * **Undercut Detection**: Uses a voxel-based approach to identify geometry that would be trapped in a simple two-part mold.
  * **Draft Angle Check**: Verifies that vertical faces have sufficient taper for mold release (checking normals against the Z-axis).
  * **Size Limits**: Prevents parts from being too small (needles) or too large (over 1m) for standard molding equipment.
* **Actions**:
  * `Box`: Create a parametric rectangular solid.
  * `IM_Hole`: Create specialized holes for injection molded parts.
  * `Draft`: Apply draft angles (essential for mold release).
  * `Fillet` / `Chamfer`: Edge treatments for stress reduction and aesthetics.

### B. Assembly Workbench

Handles the organization of multiple parts into a single product.

* **Constraints**:
  * **Geometric Intersection**: Collision detection to ensure parts do not overlap.
  * **Assembly Weight**: Total mass calculated using volume and material density.
  * **Total Cost**: Calculated using material volume and manufacturing cost per material type.
* **Actions**:
  * `PlacePart`: Position a specific component or sub-assembly within the global coordinate system.
  * `RemovePart`: Delete a component from the assembly.

### C. Sheet Metal Workbench

(Work in progress/Planned) For designing brake-formed sheet metal components.

* **Actions**: `CreateFace`, stamping, and folding operations.

## 4. Observation Space

The agent receives a multi-modal observation:

* **Product Request (Goal)**:
  * `max_assembly_cost`: Economic limit.
  * `max_assembly_weight`: Mass limit.
  * `max_size`: Bounding box constraints.
  * `Geometry Objectives`: A list of target shapes (AABB, Spheres) that must be "hit" or avoided by the design. Includes `is_reward` (positive vs forbidden zones) and `is_goal_inside`.
* **Assembly State**:
  * Current geometry representation (Build123d Compound objects).
  * Calculated properties (current total weight, current total cost).
* **Simulation Data**:
  * Feedback from physical solvers (e.g., displacement, stress, flow).
  * Handled via an asynchronous queue (Skypilot integration).
* **Status Indicators**:
  * `step_count`, `number_of_simulations`, `is_first/last/terminal`.
  * `checks`: Detailed results of manufacturability tests (e.g., `are_costs_met`, `has_no_undercuts`, `is_in_bounds`).

## 5. Action Space

Actions are parametric and discrete-continuous hybrids:

* **Discrete Component**: Selecting the action type (e.g., `ActionIDs.Box`) and the target workbench.
* **Continuous Component**: Parameters for the operation (e.g., `x, y, z` coordinates, `width, height, depth`, `radius`, `angle`).

## 6. Reward Mechanism

The reward function is designed to balance performance with efficiency:

* **Objective Reward (+)**: Reward proportional to the percentage of geometry objectives met (e.g., a part filling a target volume).
* **Modelling Penalty (-)**: Small penalties for actions that result in modeling errors or invalid geometry.
* **Constraint Penalty (-)**: Specific penalties for exceeding weight, cost, or size limits.
* **Simulation Cost (-)**: A fixed penalty (e.g., -0.5) for every call to the physics simulator, encouraging the agent to "think" (model accurately) before "testing" (simulating).
* **Termination Reward**: A large reward (e.g., +10) for successfully completing all objectives within the simulation budget.

## 7. Technical Implementation

* **Engine**: Built on `gymnax` for JAX compatibility, enabling hardware-accelerated RL training.
* **CAD Kernel**: `Build123d` / `OCP` (OpenCASCADE) for robust geometric operations.
* **Compute Orchestration**: Uses `Skypilot` (sky.Task) to manage simulation jobs on remote workers, ensuring that expensive physics computations do not bottleneck the RL training loop.
