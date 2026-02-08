# Desired Architecture: WP6 - Production & Enterprise

## Objective of the system

To transform the experimental "Problemologist" into a robust, enterprise-grade engineering platform. This focuses on reliability, security, extended analysis, and "Code on Chip" capabilities. We move from "It works on my machine" to "It works in the field for 10 years."

### Goals

1. **Reliability Engineering**: Move beyond deterministic "It works once" to probabilistic "It works 99.9% of the time" (Six Sigma mindset).
2. **Thermal Analysis**: Ensure designs don't overheat under continuous load.
3. **Code on Chip**: Compile agent logic into microcontroller firmware (C/C++) for deployment.
4. **Security**: Protect IP and ensure safe execution of generated code via signing.

## Reliability Analysis (Monte Carlo Simulation)

Engineering is the management of uncertainty. The agent must characterize this uncertainty.

### Workflow: The "batch-validation" Tool

1. **Parameter Definition**: The agent defines tolerances.
    * `Shaft Diameter ~ N(10.0, 0.05)` (Normal distribution, mean 10, std dev 0.05).
    * `Load ~ U(90, 110)` (Uniform distribution).
2. **Sampling**: We use Latin Hypercube Sampling (LHS) to generate $N=1000$ unique configuration vectors.
3. **Execution**: We spawn 1000 parallel simulations on the worker cluster.
    * *Infrastructure*: This requires **Kubernetes Jobs** (K8s). Docker Compose is insufficient.
4. **Analysis**:
    * Count failures ($N_{fail}$).
    * Calculate Reliability $R = 1 - (N_{fail}/N)$.
    * Identify "Sensitivity": Which parameter caused the most failures? (e.g., "The design is highly sensitive to shaft diameter").

## Thermal Optimization

We introduce a **Thermal Solver** (Heat Transfer FEM).

### Physics Integration

* **Solver**: **Elmer** or **FEniCS** (Heat Equation).
* **Coupling**: One-way coupling usually suffices (Heat doesn't deform part significantly, but part geometry affects heat).

### Inputs

1. **Material Properties**: Thermal Conductivity ($k$), Density ($\rho$), Specific Heat ($C_p$).
2. **Sources**: Heat generation ($Q$ in Watts). "The Motor produces 50W of heat."
3. **Sinks**: Convection ($h$ coefficient) to ambient air. "Air cooling at $20^\circ C$."

### Validation Metric

* `Max(Temperature) < T_limit (e.g. 80C)`.

## Code on Chip (Firmware generation)

The ultimate goal involves not just simulating, but *deploying* the logic to hardware.

### The Pipeline

1. **Logic Definition**: The agent writes high-level Python logic (State Machines, PID controllers).
2. **Transpilation**: We use **MicroPython** (easiest) or **Transcrypt/Cython** to convert to C.
3. **Board Support Package (BSP)**: The agent selects a target (e.g., `STM32F4-Discovery`).
    * We maintain a mapping of "Abstract Pin" (`Motor_PWM`) to "Physical Pin" (`PA8`).
4. **Compilation**: We use **PlatformIO Core** (CLI) running in a container.
    * `pio run -e stm32f4`
5. **Artifact**: The system produces a `.bin` or `.hex` file.
6. **Simulation**: We load this binary into **Renode** (an emulator) to verify it boots and toggles pins.

## Security & IP

### Code Signing and Chain of Custody

In an enterprise environment, we must know *who* (or what) designed a part.

1. **Identity**: Each Agent Instance has a private cryptographic key.
2. **Signing**: Every artifact (STEP, PDF, HEX) is signed.
    * `design.step.sig`
3. **Audit Log**: The signature includes a hash of the *Prompt* and *Context* used to generate it. This prevents "Black Box" liability.

### Sandboxing (Tier 2)

For running user-provided code or untrusted 3rd party optimization scripts, we upgrade from Docker to **gVisor** or **Firecracker MicroVMs**. This provides kernel-level isolation.

## Tech Stack

* **Kubernetes (K8s) Jobs**: Required for managing the massive parallelism of Monte Carlo (1000+ pods).
* **PlatformIO**: The de-facto standard for command-line embedded development. Supports 1000+ boards.
* **Renode**: Antmicro's open source emulator for multinode systems.
* **Elmer / OpenFEMA**: Open Source solvers for Thermal/Multiphysics.
* **Salib**: Python Sensitivity Analysis Library (Sobol indices) for analyzing Monte Carlo results.

## Limitations

1. **Time**: Monte Carlo takes time. 1000 sims x 10 seconds = 3 CPU-hours. Cost impact must be managed.
2. **Firmware Complexity**: We can compile simple logic. We cannot yet compile complex RTOS-based, multi-threaded applications with networking.
