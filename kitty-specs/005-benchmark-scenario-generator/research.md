# Research: Benchmark Scenario Generator

**Status**: Complete
**Conclusion**: No external research required.

## Decisions

* **Orchestration**: **`deepagents`** (Controller/Worker). The Benchmark Generator runs as a specialized graph on the Controller, delegating code execution and rendering to distributed workers.
* **Isolation**: Every generation attempt/variation runs in a **Podman** sandbox.
* **Persistence**: Benchmarks (code and MJCF) are stored as **Assets** on S3.
* **Verification**: The "Big Bang" stability test runs on the worker node for safety.

## Implementation References

* **Sandbox Orchestration**: See `deepagents` `SandboxFilesystemBackend`.
* **Distributed Tools**: See utility scripts in the `utils` folder available on workers.
