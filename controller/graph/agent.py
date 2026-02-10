from langchain_openai import ChatOpenAI
from deepagents import create_deep_agent

from controller.config.settings import settings
from controller.clients.backend import RemoteFilesystemBackend
from controller.observability.langfuse import get_langfuse_callback
from controller.prompts import get_prompt
from shared.logging import get_logger

logger = get_logger(__name__)


def create_agent_graph(
    backend: RemoteFilesystemBackend, agent_name: str = "engineer_coder"
):
    """Create a Deep Agent graph with remote filesystem backend."""

    llm = ChatOpenAI(
        model=settings.llm_model,
        temperature=settings.llm_temperature,
        base_url=settings.openai_api_base,
    )

    # Try to get Langfuse callback
    langfuse_callback = get_langfuse_callback()
    callbacks = [langfuse_callback] if langfuse_callback else []

    # Map simple agent names to config prompt keys
    # Keys must match controller/config/prompts.yaml structure
    prompt_mapping = {
        "benchmark_planner": "benchmark_generator.planner.system",
        "benchmark_coder": "benchmark_generator.coder.system",
        "benchmark_reviewer": "benchmark_generator.reviewer.system",
        "engineer_planner": "engineer.planner.system",
        "engineer_coder": "engineer.engineer.system",
        "engineer_reviewer": "engineer.critic.system",
    }

    # Fallback or direct key usage
    prompt_key = prompt_mapping.get(agent_name, f"{agent_name}.system")

    try:
        system_prompt = get_prompt(prompt_key)
    except Exception as err:
        raise ValueError(
            f"Could not find prompt for {agent_name} mapped to {prompt_key}, trying fallback."
        ) from err

    if callbacks:
        llm = llm.with_config({"callbacks": callbacks})

    agent = create_deep_agent(
        model=llm,
        backend=backend,
        system_prompt=system_prompt,
        name=agent_name,
    )
    return agent, langfuse_callback
