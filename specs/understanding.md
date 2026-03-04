# Internal notes on understanding how system actually works (for humans by humans)

Read these in this order for the current LLM/agent implementation.

1) Top-Level Flow

  - specs/desired_architecture.md
  - controller/api/main.py
  - controller/api/tasks.py
  - controller/graph/agent.py

2) Engineer Agent Graph + Nodes

  - controller/agent/graph.py
  - controller/agent/state.py
  - controller/agent/nodes/base.py
  - controller/agent/nodes/planner.py
  - controller/agent/nodes/coder.py
  - controller/agent/nodes/plan_reviewer.py
  - controller/agent/nodes/electronics_planner.py
  - controller/agent/nodes/electronics_engineer.py
  - controller/agent/nodes/electronics_reviewer.py
  - controller/agent/nodes/execution_reviewer.py
  - controller/agent/nodes/skills.py
  - controller/agent/nodes/summarizer.py
  - controller/agent/nodes/cots_search.py

3) Benchmark Generator Graph + Nodes

  - controller/agent/benchmark/graph.py
  - controller/agent/benchmark/state.py
  - controller/agent/benchmark/nodes.py
  - controller/agent/benchmark/models.py
  - controller/agent/benchmark/tools.py

4) Prompting + LLM Runtime

  - config/prompts.yaml
  - controller/prompts.py
  - controller/agent/prompt_manager.py
  - controller/agent/config.py
  - controller/config/settings.py
  - controller/agent/dspy_utils.py
  - controller/agent/mock_llm.py

5) Tooling + Worker Boundary

  - controller/agent/tools.py
  - controller/middleware/remote_fs.py
  - controller/clients/worker.py
  - worker_light/api/routes.py
  - worker_light/runtime/executor.py
  - shared/workers/filesystem/policy.py
  - config/agents_config.yaml

6) Data Contracts + Persistence

  - shared/enums.py
  - shared/models/schemas.py
  - shared/workers/schema.py
  - controller/persistence/models.py
  - controller/agent/initialization.py
  - shared/assets/template_repos/engineer/plan.md
  - shared/assets/template_repos/benchmark_generator/plan.md

7) Steerability / Human-in-the-loop

  - controller/graph/steerability_node.py
  - controller/services/steerability/service.py
  - controller/api/routes/steerability.py
  - shared/models/steerability.py

8) COTS Subagent

  - shared/cots/agent.py

9) Behavior-Defining Tests (very useful)

  - tests/agent/test_graph.py
  - tests/agent/test_prompt_manager.py
  - tests/agent/test_steerability.py
  - tests/test_controller_tasks.py
  - tests/controller/test_agent_graph.py
  - tests/observability/test_tracing_interaction.py
  - tests/cots/test_search_agent.py
  - tests/unit/controller/agent/test_agent_mock_integration.py
