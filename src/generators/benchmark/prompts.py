from src.agent.utils.prompts import get_prompt

PLANNER_PROMPT = get_prompt("benchmark_generator.planner")
CODER_PROMPT = get_prompt("benchmark_generator.coder")
CRITIC_PROMPT = get_prompt("benchmark_generator.critic")
FIXER_PROMPT = get_prompt("benchmark_generator.fixer")
