from langchain_openai import ChatOpenAI
from deepagents import create_deep_agent

from controller.config.settings import settings
from controller.clients.backend import RemoteFilesystemBackend
from controller.observability.langfuse import get_langfuse_callback
from controller.prompts import get_prompt
from shared.logging import get_logger

logger = get_logger(__name__)


def create_agent_graph(backend: RemoteFilesystemBackend):
    """Create a Deep Agent graph with remote filesystem backend."""

    llm = ChatOpenAI(
        model=settings.llm_model,
        temperature=settings.llm_temperature,
        base_url=settings.openai_api_base,
    )

    # Try to get Langfuse callback
    langfuse_callback = get_langfuse_callback()
    callbacks = [langfuse_callback] if langfuse_callback else []

    # System prompt from config (Strict, will raise if missing)
    system_prompt = get_prompt("engineer.planner.system")

    if callbacks:
        llm = llm.with_config({"callbacks": callbacks})

    return create_deep_agent(
        model=llm,
        backend=backend,
        system_prompt=system_prompt,
        name="CAD Agent",
    )
