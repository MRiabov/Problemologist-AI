# Research: Benchmark Scenario Generator

**Status**: Complete
**Conclusion**: No external research required.

## Decisions

*   **Orchestration**: `deepagents` (LangGraph). Already researched in `002-vlm-cad-agent`.
*   **Physics**: `mujoco`. Standard for project.
*   **CAD**: `build123d`. Standard for project.

## Implementation References

*   **Validator Logic**: See `src/compiler/sim_client.py` (from WP03) for examples of loading MJCF strings.
*   **LLM Tools**: See `src/agent/utils/llm.py` for model instantiation.
