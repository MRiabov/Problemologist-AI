from src.agent.utils.prompts import get_prompt

PLANNER_PROMPT = get_prompt("benchmark_generator.planner")
CODER_PROMPT = get_prompt("benchmark_generator.coder")
CRITIC_PROMPT = get_prompt("benchmark_generator.critic")
FIXER_PROMPT = get_prompt("benchmark_generator.fixer")

LINTER_FEEDBACK_PREFIX = """The following errors were found by static analysis (Ruff/Pyrefly). 
These MUST be fixed before the code can be executed in the simulator.
Static Analysis Errors:
"""
