from deepagents import create_deep_agent
from langchain_openai import ChatOpenAI

from controller.clients.backend import RemoteFilesystemBackend
from controller.config.settings import settings
from controller.observability.langfuse import get_langfuse_callback
from controller.prompts import get_prompt
from shared.cots.agent import search_cots_catalog
from shared.logging import get_logger

logger = get_logger(__name__)


def create_agent_graph(
    backend: RemoteFilesystemBackend,
    agent_name: str = "engineer_coder",
    trace_id: str | None = None,
):
    """Create a Deep Agent graph with remote filesystem backend."""

    if settings.is_integration_test:
        from typing import Any

        from langchain_core.language_models.chat_models import BaseChatModel
        from langchain_core.messages import AIMessage, BaseMessage
        from langchain_core.outputs import ChatGenerationChunk, ChatResult

        class FakeModelWithTools(BaseChatModel):
            responses: list[str]
            model_name: str = "mock-model"
            _current_response_idx: int = 0

            def _generate(
                self,
                messages: list[BaseMessage],
                stop: list[str] | None = None,
                run_manager: Any = None,
                **kwargs: Any,
            ) -> ChatResult:
                _ = messages, stop, run_manager, kwargs
                if self._current_response_idx >= len(self.responses):
                    raise ValueError(
                        "No more responses available in FakeModelWithTools"
                    )
                response_content = self.responses[self._current_response_idx]
                self._current_response_idx += 1
                return ChatResult(
                    generations=[
                        ChatGenerationChunk(message=AIMessage(content=response_content))
                    ]
                )

            @property
            def _llm_type(self) -> str:
                return "fake-chat-model"

            def bind_tools(self, tools: Any, **kwargs: Any) -> Any:
                _ = tools, kwargs
                return self

            async def ainvoke(self, input_data, config=None, **kwargs):
                import asyncio

                await asyncio.sleep(
                    1.0
                )  # Simulate some processing time for interruption
                return await super().ainvoke(input_data, config, **kwargs)

        # Responses that perform some basic tool calls to satisfy tests
        # We provide a generous sequence of tool calls and completions
        responses = [
            '{"action": "write_file", "action_input": {"path": "worker_execution.txt", "content": "verified"}}',
            '{"action": "submit_for_review", "action_input": {"script_path": "solution.py"}}',
            "I have completed the task successfully.",
            # Add more for potential retries or subagent calls
            '{"action": "write_file", "action_input": {"path": "plan.md", "content": "## 1. Solution Overview\\nDone."}}',
            '{"action": "write_file", "action_input": {"path": "todo.md", "content": "- [x] Task"}}',
            '{"action": "write_file", "action_input": {"path": "objectives.yaml", "content": "objectives: {}"}}',
            '{"action": "submit_for_review", "action_input": {"script_path": "solution.py"}}',
            "Handover complete.",
        ]
        llm = FakeModelWithTools(responses=responses)
    else:
        llm = ChatOpenAI(
            model_name=settings.llm_model,
            temperature=settings.llm_temperature,
            base_url=settings.openai_api_base,
            api_key=settings.openai_api_key,
        )

    # Try to get Langfuse callback
    langfuse_callback = get_langfuse_callback(trace_id=trace_id)
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
        "cots_search": "subagents.cots_search.system",
    }

    # Fallback or direct key usage
    prompt_key = prompt_mapping.get(agent_name, f"{agent_name}.system")

    try:
        system_prompt = get_prompt(prompt_key)
    except Exception as err:
        raise ValueError(
            f"Could not find prompt for {agent_name} mapped to {prompt_key}."
        ) from err

    if callbacks:
        llm = llm.with_config({"callbacks": callbacks})

    # Define subagents
    cots_search_subagent = {
        "name": "cots_search",
        "description": "Search for components (motors, fasteners, bearings).",
        "system_prompt": get_prompt("subagents.cots_search.system"),
        "tools": [search_cots_catalog],
    }

    # Map agents that have access to subagents
    primary_agents = [
        "engineer_planner",
        "engineer_coder",
        "benchmark_planner",
    ]
    subagents = []
    if agent_name in primary_agents:
        subagents = [cots_search_subagent]

    # Tools for the agent itself
    agent_tools = []
    if agent_name == "cots_search":
        agent_tools = [search_cots_catalog]

    agent = create_deep_agent(
        model=llm,
        backend=backend,
        system_prompt=system_prompt,
        name=agent_name,
        subagents=subagents,
        tools=agent_tools,
    )
    return agent, langfuse_callback
