# Analysis of Tech Debt and Missing Abstractions

Following the identification of significant abstractions, I've analyzed the codebase for "tech debt" and areas where the lack of abstractions leads to unnecessary complexity and code bloat.

## 1. "God Object" Bloat: `SimulationLoop`

The [SimulationLoop](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/simulation/loop.py) is a prime example of an "insignificant abstraction" that has become a dumping ground for logic. At ~850 lines, it handles:

- **Environment Setup**: Backend initialization and scene loading.
- **Manufacturing Validation**: Direct calls to `validate_and_price` and DFM logic.
- **Electronics Simulation**: Inlined SPICE simulation and a 60-line BFS fallback for connectivity.
- **Physics Loops**: Collision checking, stress monitoring, and fluid dynamics.
- **Reporting**: Event emission and metric gathering.

> [!IMPORTANT]
> **Refactor Opportunity**: Split this into a `SimulationRunner` and separate specialists like `ElectronicsManager`, `MetricCollector`, and `SuccessEvaluator`.

## 2. Missing Base Abstractions (Logic Duplication)

### Simulation Builders

Both [MuJoCoSimulationBuilder](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/simulation/builder.py#L448) and [GenesisSimulationBuilder](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/simulation/builder.py#L704) re-implement the same logic for traversing an assembly, resolving positions, and handling zones.

- **Problem**: 60% of the code in these classes is identical.
- **Fix**: Extract a `CommonAssemblyTraverser` to provide the unified part data to backend-specific emitters.

### Filesystem Backends

The [SandboxFilesystemBackend](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/filesystem/backend.py#L72) (S3) and [LocalFilesystemBackend](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/filesystem/backend.py#L402) share almost all high-level logic (error handling, formatting, line numbering).

- **Problem**: Every change to formatting or validation must be applied twice.
- **Fix**: Use a Bridge or Decorator pattern where common logic resides in a base class and only the specific `read_bytes`/`write_bytes` calls are overridden.

## 3. Boilerplate in Agent Nodes

Every agent node (e.g., [PlannerNode](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/planner.py), [Benchmark Nodes](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py)) manually initializes its own `WorkerClient`, `RemoteFilesystemMiddleware`, and `DatabaseCallbackHandler`.

- **Problem**: Dozens of lines of setup boilerplate per node.
- **Fix**: Use a `SharedNodeContext` or an `AgentFactory` to inject these dependencies into the node execution.

## 4. Manual XML/JSON Handling

The codebase relies heavily on manual XML construction in `SceneCompiler` and regex-based extraction of LLM outputs.

- **Problem**: Brittle and difficult to maintain.
- **Fix**: Use declarative Pydantic-based scene definitions and structured output parsing from LangChain.

## Summary of SLOC Impact

The 75K SLOC bloat is largely driven by this "copy-paste-modify" pattern and the inclusion of large, monolithic logic blocks in `loop.py` and various builders. Moving to shared abstractions could likely reduce the core codebase size by 20-30%.
