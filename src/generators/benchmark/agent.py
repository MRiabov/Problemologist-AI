from src.agent.utils.config import Config
from src.agent.graph.graph import build_graph
from src.agent.tools.benchmark_tools import (
    run_benchmark_linter,
    validate_benchmark_model,
)
from src.agent.tools.env import read_script

# Constants used by dashboard
MAX_ATTEMPTS = Config.MAX_STEPS


# Helper to build the configured agent
def build_benchmark_agent():
    """
    Builds the VLM CAD Agent configured for Benchmark Generation.
    It injects:
    1. Benchmark-specific tools (linter, validator).
    2. Benchmark-specific system prompts (via runtime_config).
    """
    tools = [run_benchmark_linter, validate_benchmark_model, read_script]
    return build_graph(
        extra_tools=tools, validation_tool_name="validate_benchmark_model"
    )


generator_agent = build_benchmark_agent().compile()

# Default configuration to be passed at runtime
DEFAULT_RUNTIME_CONFIG = {
    "system_prompt_overrides": {
        "planner": "benchmark_generator.planner",
        "actor": "benchmark_generator.coder",
        "critic": "benchmark_generator.critic",
    }
}
