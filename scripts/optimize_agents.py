import argparse
import os
import dspy
import structlog
from dspy.teleprompt import GEPA

from controller.agent.benchmark.data_loader import load_benchmark_dataset
from controller.agent.bootstrap import build_eval_program
from controller.agent.dspy_utils import cad_simulation_metric
from controller.agent.reward import load_reward_config

logger = structlog.get_logger(__name__)


def optimize_agent(
    agent_name: str,
    trainset_size: int = 10,
    max_bootstrapped_demos: int = 3,
    use_gepa: bool = True,
) -> dspy.Module:
    """
    Runs the DSPy optimization pipeline for a specific agent.
    """
    logger.info(
        "optimization_start",
        agent_name=agent_name,
        method="GEPA" if use_gepa else "BootstrapFewShot",
    )

    # 1. Load Dataset
    dataset = load_benchmark_dataset(agent_name)
    if not dataset:
        logger.error("empty_dataset", agent_name=agent_name)
        return None

    trainset = dataset[:trainset_size]
    # GEPA doesn't strictly need a valset but it helps if available
    valset = (
        dataset[trainset_size : trainset_size + 5]
        if len(dataset) > trainset_size
        else None
    )

    # 2. Setup Program (Module)
    program = build_eval_program(agent_name)

    # 3. Setup Teleprompter (Optimizer)
    if use_gepa:
        # GEPA (Genetic-Pareto) uses reflective feedback from rollouts
        # to evolve the prompt instructions.
        reflection_model = os.getenv(
            "GEPA_REFLECTION_MODEL", "stepfun/step-3.5-flash:free"
        )
        logger.info("configuring_gepa", reflection_model=reflection_model)

        teleprompter = GEPA(
            metric=cad_simulation_metric,
            auto="light",  # "light" is usually enough for prompt evolution
            reflection_lm=dspy.LM(reflection_model) if reflection_model else None,
            num_threads=1,  # Keep it simple for simulation-heavy tasks
        )
    else:
        # Fallback to standard BootstrapFewShot
        from dspy.teleprompt import BootstrapFewShot

        teleprompter = BootstrapFewShot(
            metric=cad_simulation_metric,
            max_bootstrapped_demos=max_bootstrapped_demos,
        )

    # 4. Run Optimization
    logger.info("optimization_compile_start")
    compiled_program = teleprompter.compile(
        program,
        trainset=trainset,
        **({"valset": valset} if use_gepa and valset else {}),
    )

    # 5. Save Compiled Program
    save_path = f"config/compiled_prompts/{agent_name}.json"
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    compiled_program.save(save_path)

    logger.info("optimization_complete", agent_name=agent_name, save_path=save_path)
    return compiled_program


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DSPy Agent Optimizer (GEPA)")
    parser.add_argument(
        "--agent", type=str, required=True, help="Agent name (e.g., cad_engineer)"
    )
    parser.add_argument("--size", type=int, default=10, help="Trainset size")
    parser.add_argument(
        "--demos", type=int, default=3, help="Max bootstrapped demos (if using BF)"
    )
    parser.add_argument(
        "--no-gepa", action="store_true", help="Disable GEPA and use BootstrapFewShot"
    )

    args = parser.parse_args()

    # Ensure LLM is configured globally if not already
    if not dspy.settings.lm:
        # Default to a reliable model from env or settings
        model = os.getenv("DSPY_OPTIMIZER_MODEL", "stepfun/step-3.5-flash:free")
        dspy.settings.configure(lm=dspy.LM(model))

    optimize_agent(args.agent, args.size, args.demos, use_gepa=not args.no_gepa)
