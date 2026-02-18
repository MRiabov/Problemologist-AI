# Specification: Potential Worker Infrastructure Split

This document captures the feasibility and proposed architecture for decoupling the Problemologist Worker into specialized components to improve scalability and responsiveness.

## 1. Problem Statement

The current monolithic worker architecture suffers from:

- **Sequential Blockage**: A global `SIMULATION_SEMAPHORE` blocks lightweight operations (like linting) during long-running simulations.
- **Resource Inefficiency**: Scaling workers means duplicating heavy physics dependencies (Genesis, MuJoCo) even for nodes that only handle filesystem tasks. (torch and genesis add 10GB to image)
- **Global Lock Latency**: Genesis global state initialization and locks make intra-process concurrency difficult.

## 2. Proposed Specialized Architecture

### Worker-Light (Admin & Tools)

- **Role**: Lightweight API gateway for the controller.
- **Responsibilities**:
  - Filesystem operations (Read/Write/Edit).
  - Git operations.
  - Code linting and style checking.
  - Topology analysis and geometric validation.
- **Resource Profile**: Low CPU/RAM.

### Worker-Heavy (Simulation & rendering)

- **Role**: Resource-intensive execution engine.
- **Responsibilities**:
  - Build physics scenes from CAD scripts.
  - Execute physics simulations.
  - Render video and stress heatmaps.
- **Resource Profile**: High GPU/VRAM.

---

## 3. Distributed/Stateless Design Patterns

### Stateless Simulation Payloads

To avoid the "PITA" of shared filesystems (NFS/EFS) in distributed hyperscalers (e.g., AWS Spot Instances), the system should transition to **Stateless Simulation**:

1. **Controller/Worker-Light** bundles the build script and configuration into a self-contained payload (e.g., JSON or gzipped bundle).
2. **Worker-Heavy** receives the payload, unpacks it into ephemeral local storage (`/tmp`), and runs the simulation.
3. **Result**: No shared session state is required between distributed containers.

### Genesis JIT Kernel Caching

Genesis uses Just-In-Time (JIT) compilation for GPU kernels. To maintain performance in a distributed environment:

- **Pre-warmed Images**: Common benchmark topologies should be "baked" into the Docker image layers during CI.
- **Lazy Cloud Sync**: Use a sidecar or startup/shutdown script to sync `~/.cache/genesis` with an S3 bucket (content-addressed by GPU hash).
<!-- Note: I'm not sure about this... the cache can be large and faster to simply regenerate? -->
- **Result**: New instances "boot" with the collective compilation memory of the fleet.

---

## 4. Implementation Readiness

This split is considered **High Reward / Medium Risk**. The primary risk is the initial transition of the `WorkerClient` to handle dual-URL routing and payload-based data forwarding instead of path-based references.

---

## 5. References

- Initial Research: [feasibility_report.md](file:///home/maksym/.gemini/antigravity/brain/54b58243-60a9-416c-b1d5-9ca967e1c213/feasibility_report.md)
- Draft Implementation Plan: [implementation_plan.md](file:///home/maksym/.gemini/antigravity/brain/54b58243-60a9-416c-b1d5-9ca967e1c213/implementation_plan.md)
