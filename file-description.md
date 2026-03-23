.
├── ./alembic.ini # Configuration for Alembic database migrations.
├── ./build.log # Log file for build processes.
├── ./config # Global configuration directory.
│   ├── ./config/generator_config.yaml # Configuration for the benchmark generator agent.
│   ├── ./config/lint_config.yaml # Linter settings (ruff/pyright).
│   ├── ./config/manufacturing_config.yaml # Settings for manufacturing validation workbenches.
│   └── ./config/prompts.yaml # System prompts for LLM agents.
├── ./controller # The 'Brain': Orchestrates agents, manages state, and handles API requests.
│   ├── ./controller/activities # Temporal activities definitions.
│   │   ├── ./controller/activities/execution.py # Execution workflow activities.
│   │   ├── ./controller/activities/__init__.py # Package initialization file.
│   │   └── ./controller/activities/simulation.py # Simulation workflow activities.
│   ├── ./controller/agent # Agent graph definitions (LangGraph).
│   │   ├── ./controller/agent/benchmark # Benchmark generator agent specific code.
│   │   │   ├── ./controller/agent/benchmark/graph.py # Python source code.
│   │   │   ├── ./controller/agent/benchmark/__init__.py # Package initialization file.
│   │   │   ├── ./controller/agent/benchmark/models.py # Python source code.
│   │   │   ├── ./controller/agent/benchmark/nodes.py # Python source code.
│   │   │   ├── ./controller/agent/benchmark/schema.py # Python source code.
│   │   │   ├── ./controller/agent/benchmark/state.py # Python source code.
│   │   │   ├── ./controller/agent/benchmark/storage.py # Python source code.
│   │   │   ├── ./controller/agent/benchmark/templates
│   │   │   │   ├── ./controller/agent/benchmark/templates/coder_prompt.txt
│   │   │   │   ├── ./controller/agent/benchmark/templates/__init__.py # Package initialization file.
│   │   │   │   ├── ./controller/agent/benchmark/templates/planner_prompt.txt
│   │   │   │   └── ./controller/agent/benchmark/templates/reviewer_prompt.txt
│   │   │   └── ./controller/agent/benchmark/utils
│   │   │   └── ./controller/agent/benchmark/utils/__init__.py # Package initialization file.
│   │   ├── ./controller/agent/config.py # Python source code.
│   │   ├── ./controller/agent/graph.py # Python source code.
│   │   ├── ./controller/agent/__init__.py # Package initialization file.
│   │   ├── ./controller/agent/nodes # Individual graph nodes (Architect, Critic, Engineer).
│   │   │   ├── ./controller/agent/nodes/architect.py # Python source code.
│   │   │   ├── ./controller/agent/nodes/critic.py # Python source code.
│   │   │   ├── ./controller/agent/nodes/engineer.py # Python source code.
│   │   │   ├── ./controller/agent/nodes/__init__.py # Package initialization file.
│   │   │   └── ./controller/agent/nodes/sidecar.py # Python source code.
│   │   ├── ./controller/agent/prompt_manager.py # Python source code.
│   │   ├── ./controller/agent/run.py # Python source code.
│   │   └── ./controller/agent/state.py # Python source code.
│   ├── ./controller/api # FastAPI routes and application definition.
│   │   ├── ./controller/api/__init__.py # Package initialization file.
│   │   ├── ./controller/api/main.py # Python source code.
│   │   ├── ./controller/api/manager.py # Python source code.
│   │   ├── ./controller/api/ops.py # Python source code.
│   │   └── ./controller/api/routes
│   │   ├── ./controller/api/routes/episodes.py # API endpoints for managing episodes.
│   │   └── ./controller/api/routes/skills.py # Python source code.
│   ├── ./controller/clients # Clients for external services and worker communication.
│   │   ├── ./controller/clients/backend.py # Python source code.
│   │   ├── ./controller/clients/__init__.py # Package initialization file.
│   │   └── ./controller/clients/worker.py # Python source code.
│   ├── ./controller/config
│   │   └── ./controller/config/settings.py # Python source code.
│   ├── ./controller/Dockerfile
│   ├── ./controller/graph
│   │   ├── ./controller/graph/agent.py # Python source code.
│   │   └── ./controller/graph/__init__.py # Package initialization file.
│   ├── ./controller/__init__.py # Package initialization file.
│   ├── ./controller/middleware # Middleware for `deepagents` (e.g., remote filesystem).
│   │   └── ./controller/middleware/remote_fs.py # Python source code.
│   ├── ./controller/migrations # Database migration scripts (Alembic).
│   │   ├── ./controller/migrations/env.py # Python source code.
│   │   ├── ./controller/migrations/README
│   │   ├── ./controller/migrations/script.py.mako
│   │   └── ./controller/migrations/versions
│   │   ├── ./controller/migrations/versions/5b673a27de1b_initial_benchmark_models.py # Python source code.
│   │   ├── ./controller/migrations/versions/b5f3ce2a7b6d_add_episode_trace_and_asset_models.py # Python source code.
│   │   └── ./controller/migrations/versions/__init__.py # Package initialization file.
│   ├── ./controller/observability # Tracing (LangFuse), logging, and metrics.
│   │   ├── ./controller/observability/broadcast.py # Python source code.
│   │   ├── ./controller/observability/database.py # Python source code.
│   │   ├── ./controller/observability/__init__.py # Package initialization file.
│   │   └── ./controller/observability/langfuse.py # Python source code.
│   ├── ./controller/persistence # Database models and access layer (SQLAlchemy).
│   │   ├── ./controller/persistence/db.py # Python source code.
│   │   ├── ./controller/persistence/__init__.py # Package initialization file.
│   │   └── ./controller/persistence/models.py # Python source code.
│   ├── ./controller/prompts.py # Python source code.
│   ├── ./controller/temporal_worker.py # Worker entry point for Temporal workflows.
│   ├── ./controller/tools
│   │   └── ./controller/tools/fs.py # Python source code.
│   └── ./controller/workflows # Temporal workflow definitions.
│   ├── ./controller/workflows/execution.py # Python source code.
│   └── ./controller/workflows/simulation.py # Python source code.
├── ./controller_openapi.json # OpenAPI specification for the Controller API.
├── ./debug_single_simulation.py # Python source code.
├── ./docker-compose.yml # Docker composition for local development.
├── ./docs # Project documentation and reviews.
│   ├── ./docs/code-reviews
│   │   ├── ./docs/code-reviews/code-smells-feb-3.md # Markdown documentation or specification.
│   │   ├── ./docs/code-reviews/code-smells-feb-3-round-2.md # Markdown documentation or specification.
│   │   ├── ./docs/code-reviews/code-smells-feb-3-round-3.md # Markdown documentation or specification.
│   │   ├── ./docs/code-reviews/code-smells-feb-3-round-4.md # Markdown documentation or specification.
│   │   ├── ./docs/code-reviews/code-smells-feb-3-round-5.md # Markdown documentation or specification.
│   │   ├── ./docs/code-reviews/code-smells-feb-3-round-6.md # Markdown documentation or specification.
│   │   ├── ./docs/code-reviews/code-smells-feb-3-round-7.md # Markdown documentation or specification.
│   │   └── ./docs/code-reviews/code-smells-feb-4-round-8.md # Markdown documentation or specification.
│   └── ./docs/frontend-component-review.md # Markdown documentation or specification.
├── ./file-description.md # Markdown documentation or specification.
├── ./frontend # React/Vite frontend application.
│   ├── ./frontend/components.json # JSON data or configuration.
│   ├── ./frontend/dist
│   │   ├── ./frontend/dist/assets
│   │   │   ├── ./frontend/dist/assets/index-Bi6E43-J.js
│   │   │   └── ./frontend/dist/assets/index-hoZmgtCi.css # Cascading Style Sheets.
│   │   ├── ./frontend/dist/index.html
│   │   └── ./frontend/dist/vite.svg
│   ├── ./frontend/eslint.config.js
│   ├── ./frontend/index.html
│   ├── ./frontend/openapi.json # JSON data or configuration.
│   ├── ./frontend/package.json # JSON data or configuration.
│   ├── ./frontend/package-lock.json # JSON data or configuration.
│   ├── ./frontend/postcss.config.js
│   ├── ./frontend/public
│   │   └── ./frontend/public/favicon.svg
│   ├── ./frontend/README.md # Markdown documentation or specification.
│   ├── ./frontend/src
│   │   ├── ./frontend/src/api # Auto-generated API client for the frontend.
│   │   │   ├── ./frontend/src/api/client.ts # TypeScript source code.
│   │   │   ├── ./frontend/src/api/generated
│   │   │   │   ├── ./frontend/src/api/generated/core
│   │   │   │   │   ├── ./frontend/src/api/generated/core/ApiError.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/core/ApiRequestOptions.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/core/ApiResult.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/core/CancelablePromise.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/core/OpenAPI.ts # TypeScript source code.
│   │   │   │   │   └── ./frontend/src/api/generated/core/request.ts # TypeScript source code.
│   │   │   │   ├── ./frontend/src/api/generated/index.ts # TypeScript source code.
│   │   │   │   ├── ./frontend/src/api/generated/models
│   │   │   │   │   ├── ./frontend/src/api/generated/models/AgentRunRequest.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/AssetResponse.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/AssetType.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/EpisodeResponse.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/EpisodeStatus.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/HTTPValidationError.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/SimulationRequest.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/Skill.ts # TypeScript source code.
│   │   │   │   │   ├── ./frontend/src/api/generated/models/TraceResponse.ts # TypeScript source code.
│   │   │   │   │   └── ./frontend/src/api/generated/models/ValidationError.ts # TypeScript source code.
│   │   │   │   └── ./frontend/src/api/generated/services
│   │   │   │   ├── ./frontend/src/api/generated/services/DefaultService.ts # TypeScript source code.
│   │   │   │   ├── ./frontend/src/api/generated/services/EpisodesService.ts # TypeScript source code.
│   │   │   │   └── ./frontend/src/api/generated/services/SkillsService.ts # TypeScript source code.
│   │   │   └── ./frontend/src/api/__tests__
│   │   │   └── ./frontend/src/api/__tests__/client.test.ts # TypeScript source code.
│   │   ├── ./frontend/src/App.css # Cascading Style Sheets.
│   │   ├── ./frontend/src/App.test.tsx # React component (TypeScript).
│   │   ├── ./frontend/src/App.tsx # React component (TypeScript).
│   │   ├── ./frontend/src/assets
│   │   │   └── ./frontend/src/assets/react.svg
│   │   ├── ./frontend/src/components # React components.
│   │   │   ├── ./frontend/src/components/layout
│   │   │   │   ├── ./frontend/src/components/layout/AppLayout.tsx # React component (TypeScript).
│   │   │   │   └── ./frontend/src/components/layout/Sidebar.tsx # React component (TypeScript).
│   │   │   ├── ./frontend/src/components/Layout.tsx # React component (TypeScript).
│   │   │   ├── ./frontend/src/components/shared
│   │   │   │   └── ./frontend/src/components/shared/ConnectionError.tsx # React component (TypeScript).
│   │   │   ├── ./frontend/src/components/ui
│   │   │   │   ├── ./frontend/src/components/ui/badge.tsx # React component (TypeScript).
│   │   │   │   ├── ./frontend/src/components/ui/button.tsx # React component (TypeScript).
│   │   │   │   ├── ./frontend/src/components/ui/card.tsx # React component (TypeScript).
│   │   │   │   ├── ./frontend/src/components/ui/input.tsx # React component (TypeScript).
│   │   │   │   ├── ./frontend/src/components/ui/scroll-area.tsx # React component (TypeScript).
│   │   │   │   ├── ./frontend/src/components/ui/separator.tsx # React component (TypeScript).
│   │   │   │   ├── ./frontend/src/components/ui/sheet.tsx # React component (TypeScript).
│   │   │   │   └── ./frontend/src/components/ui/tabs.tsx # React component (TypeScript).
│   │   │   ├── ./frontend/src/components/visualization
│   │   │   │   └── ./frontend/src/components/visualization/ModelViewer.tsx # React component (TypeScript).
│   │   │   └── ./frontend/src/components/workspace
│   │   │   ├── ./frontend/src/components/workspace/ArtifactView.tsx # React component (TypeScript).
│   │   │   └── ./frontend/src/components/workspace/ReasoningTraces.tsx # React component (TypeScript).
│   │   ├── ./frontend/src/context
│   │   │   ├── ./frontend/src/context/ConnectionContext.tsx # React component (TypeScript).
│   │   │   └── ./frontend/src/context/EpisodeContext.tsx # React component (TypeScript).
│   │   ├── ./frontend/src/index.css # Cascading Style Sheets.
│   │   ├── ./frontend/src/lib
│   │   │   └── ./frontend/src/lib/utils.ts # TypeScript source code.
│   │   ├── ./frontend/src/main.tsx # React component (TypeScript).
│   │   ├── ./frontend/src/pages # Frontend pages (views).
│   │   │   ├── ./frontend/src/pages/BenchmarkGeneration.tsx # React component (TypeScript).
│   │   │   ├── ./frontend/src/pages/EngineerWorkspace.tsx # React component (TypeScript).
│   │   │   └── ./frontend/src/pages/__tests__
│   │   │   ├── ./frontend/src/pages/__tests__/BenchmarkGeneration.test.tsx # React component (TypeScript).
│   │   │   └── ./frontend/src/pages/__tests__/EngineerWorkspace.test.tsx # React component (TypeScript).
│   │   └── ./frontend/src/test
│   │   └── ./frontend/src/test/setup.ts # TypeScript source code.
│   ├── ./frontend/tailwind.config.js
│   ├── ./frontend/tsconfig.app.json # JSON data or configuration.
│   ├── ./frontend/tsconfig.json # JSON data or configuration.
│   ├── ./frontend/tsconfig.node.json # JSON data or configuration.
│   ├── ./frontend/verification
│   │   ├── ./frontend/verification/screen1.png
│   │   ├── ./frontend/verification/screen2.png
│   │   └── ./frontend/verification/screen3.png
│   ├── ./frontend/vite.config.ts # TypeScript source code.
│   └── ./frontend/vitest.config.ts # TypeScript source code.
├── ./gemini_help.txt
├── ./GEMINI.md # Markdown documentation or specification.
├── ./history.db # SQLite database for local history/development.
├── ./kitty-specs # Detailed project specifications and work packages.
│   ├── ./kitty-specs/001-agentic-cad-environment
│   │   ├── ./kitty-specs/001-agentic-cad-environment/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/001-agentic-cad-environment/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/001-agentic-cad-environment/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/001-agentic-cad-environment/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/001-agentic-cad-environment/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/001-agentic-cad-environment/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks
│   │   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks/WP01-foundation-skeleton.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks/WP02-worker-core.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks/WP03-worker-api.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks/WP04-controller-deepagents.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks/WP05-domain-utils.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/001-agentic-cad-environment/tasks/WP06-temporal-orchestration.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/001-agentic-cad-environment/tasks/WP07-observability.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/001-agentic-cad-environment/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/002-vlm-cad-agent
│   │   ├── ./kitty-specs/002-vlm-cad-agent/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/deepagents-research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks
│   │   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks/WP01-foundation-state.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks/WP02-architect-node.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks/WP03-engineer-node.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks/WP04-critic-node.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks/WP05-sidecar-learner.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/002-vlm-cad-agent/tasks/WP06-graph-orchestration.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/002-vlm-cad-agent/tasks/WP07-cli-entrypoint.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/002-vlm-cad-agent/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/003-simulation-engine
│   │   ├── ./kitty-specs/003-simulation-engine/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/003-simulation-engine/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/003-simulation-engine/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/003-simulation-engine/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/003-simulation-engine/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/003-simulation-engine/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/003-simulation-engine/tasks
│   │   │   ├── ./kitty-specs/003-simulation-engine/tasks/WP01-geometry-pipeline.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/003-simulation-engine/tasks/WP02-physics-loop.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/003-simulation-engine/tasks/WP03-rendering-artifacts.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/003-simulation-engine/tasks/WP04-api-integration.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/003-simulation-engine/tasks/WP05-e2e-verification.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/003-simulation-engine/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/004-advanced-manufacturing-workbenches
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/tasks
│   │   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/tasks/WP01-foundation-and-data-models.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/tasks/WP02-cnc-workbench-implementation.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/004-advanced-manufacturing-workbenches/tasks/WP03-injection-molding-workbench-implementation.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/004-advanced-manufacturing-workbenches/tasks/WP04-agent-integration-and-facade.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/004-advanced-manufacturing-workbenches/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/005-benchmark-scenario-generator
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/005-benchmark-scenario-generator/tasks
│   │   │   ├── ./kitty-specs/005-benchmark-scenario-generator/tasks/WP01-foundation-models.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/005-benchmark-scenario-generator/tasks/WP02-validation-worker.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/005-benchmark-scenario-generator/tasks/WP03-coder-agent.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/005-benchmark-scenario-generator/tasks/WP04-planner-reviewer.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/005-benchmark-scenario-generator/tasks/WP05-integrations.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/005-benchmark-scenario-generator/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/006-cots-assembly-system
│   │   ├── ./kitty-specs/006-cots-assembly-system/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/006-cots-assembly-system/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/006-cots-assembly-system/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/006-cots-assembly-system/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/006-cots-assembly-system/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/006-cots-assembly-system/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/006-cots-assembly-system/tasks
│   │   │   ├── ./kitty-specs/006-cots-assembly-system/tasks/WP01-data-core.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/006-cots-assembly-system/tasks/WP02-indexer.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/006-cots-assembly-system/tasks/WP03-search-runtime.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/006-cots-assembly-system/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/007-agentic-cad-dashboard
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/007-agentic-cad-dashboard/tasks
│   │   │   ├── ./kitty-specs/007-agentic-cad-dashboard/tasks/WP01-dashboard-skeleton.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/007-agentic-cad-dashboard/tasks/WP02-api-integration.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/007-agentic-cad-dashboard/tasks/WP03-live-logs.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/007-agentic-cad-dashboard/tasks/WP04-3d-viewer.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/007-agentic-cad-dashboard/tasks/WP05-benchmark-wizard.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/007-agentic-cad-dashboard/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/008-observability-system
│   │   ├── ./kitty-specs/008-observability-system/data-model.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/008-observability-system/meta.json # JSON data or configuration.
│   │   ├── ./kitty-specs/008-observability-system/plan.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/008-observability-system/quickstart.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/008-observability-system/research.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/008-observability-system/spec.md # Markdown documentation or specification.
│   │   ├── ./kitty-specs/008-observability-system/tasks
│   │   │   ├── ./kitty-specs/008-observability-system/tasks/WP01-foundation-tracing.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/008-observability-system/tasks/WP02-storage-s3.md # Markdown documentation or specification.
│   │   │   ├── ./kitty-specs/008-observability-system/tasks/WP03-validation-contracts.md # Markdown documentation or specification.
│   │   │   └── ./kitty-specs/008-observability-system/tasks/WP04-operations-backups.md # Markdown documentation or specification.
│   │   └── ./kitty-specs/008-observability-system/tasks.md # Markdown documentation or specification.
│   ├── ./kitty-specs/architecture.md # Markdown documentation or specification.
│   ├── ./kitty-specs/desired_architecture.md # The target architecture specification (source of truth).
│   ├── ./kitty-specs/desired_architecture_WP1_tuning.md # Markdown documentation or specification.
│   ├── ./kitty-specs/desired_architecture_WP2_fluids.md # Markdown documentation or specification.
│   ├── ./kitty-specs/desired_architecture_WP3_electronics.md # Markdown documentation or specification.
│   ├── ./kitty-specs/desired_architecture_WP4_adaptability.md # Markdown documentation or specification.
│   ├── ./kitty-specs/desired_architecture_WP5_topology.md # Markdown documentation or specification.
│   ├── ./kitty-specs/desired_architecture_WP6_production.md # Markdown documentation or specification.
│   ├── ./kitty-specs/discrepancies.md # Markdown documentation or specification.
│   ├── ./kitty-specs/fluids_integration.md # Markdown documentation or specification.
│   └── ./kitty-specs/plan-style-traits.md # Markdown documentation or specification.
├── ./main.py # Entry point for the application.
├── ./Podmanfile.sandbox # Container definition for the sandboxed worker environment.
├── ./preview.log
├── ./project-ideas.md # Markdown documentation or specification.
├── ./pyproject.toml # Project dependencies and configuration (uv).
├── ./pyrightconfig.json # JSON data or configuration.
├── ./README.md # Project documentation and entry point.
├── ./renders # Directory for simulation renders (images/videos).
│   ├── ./renders/component.stl
│   ├── ./renders/model.step
│   ├── ./renders/render_e15_a0.png
│   ├── ./renders/render_e15_a135.png
│   ├── ./renders/render_e15_a180.png
│   ├── ./renders/render_e15_a225.png
│   ├── ./renders/render_e15_a270.png
│   ├── ./renders/render_e15_a315.png
│   ├── ./renders/render_e15_a45.png
│   ├── ./renders/render_e15_a90.png
│   ├── ./renders/render_e45_a0.png
│   ├── ./renders/render_e45_a135.png
│   ├── ./renders/render_e45_a180.png
│   ├── ./renders/render_e45_a225.png
│   ├── ./renders/render_e45_a270.png
│   ├── ./renders/render_e45_a315.png
│   ├── ./renders/render_e45_a45.png
│   ├── ./renders/render_e45_a90.png
│   ├── ./renders/render_e75_a0.png
│   ├── ./renders/render_e75_a135.png
│   ├── ./renders/render_e75_a180.png
│   ├── ./renders/render_e75_a225.png
│   ├── ./renders/render_e75_a270.png
│   ├── ./renders/render_e75_a315.png
│   ├── ./renders/render_e75_a45.png
│   ├── ./renders/render_e75_a90.png
│   ├── ./renders/review_manifest.json # JSON data or configuration.
│   └── ./renders/scene.xml
├── ./review_help.txt
├── ./roadmap.md # Markdown documentation or specification.
├── ./ruff.toml
├── ./scripts # Helper scripts for development and operations.
│   ├── ./scripts/generate_openapi.py # Python source code.
│   ├── ./scripts/run_agent.sh
│   ├── ./scripts/run_integration_tests.sh
│   ├── ./scripts/spec_kitty_multi_implement.py # Python source code.
│   ├── ./scripts/spec_kitty_progress.py # Python source code.
│   ├── ./scripts/test_import.py # Python source code.
│   └── ./scripts/throwaway
│   ├── ./scripts/throwaway/reproduce_500.py # Python source code.
│   └── ./scripts/throwaway/reproduce_async_error.py # Python source code.
├── ./shared # Code shared between Controller and Worker.
│   ├── ./shared/assets # Static assets, schemas, and templates.
│   │   ├── ./shared/assets/cots_descriptions.json # JSON data or configuration.
│   │   ├── ./shared/assets/schemas
│   │   │   └── ./shared/assets/schemas/mjcf
│   │   │   ├── ./shared/assets/schemas/mjcf/attribute_types
│   │   │   │   └── ./shared/assets/schemas/mjcf/attribute_types/attribute_types.xsd
│   │   │   ├── ./shared/assets/schemas/mjcf/element_types
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/actuator.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/asset.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/body.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/compiler.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/contact.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/custom.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/default.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/equality.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/keyframe.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/option.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/root.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/sensor.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/size.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/statistic.xsd
│   │   │   │   ├── ./shared/assets/schemas/mjcf/element_types/tendon.xsd
│   │   │   │   └── ./shared/assets/schemas/mjcf/element_types/visual.xsd
│   │   │   ├── ./shared/assets/schemas/mjcf/example.xml
│   │   │   ├── ./shared/assets/schemas/mjcf/LICENSE
│   │   │   ├── ./shared/assets/schemas/mjcf/mujoco.xsd
│   │   │   └── ./shared/assets/schemas/mjcf/README.md # Markdown documentation or specification.
│   │   └── ./shared/assets/template_repos
│   │   ├── ./shared/assets/template_repos/benchmark_generator
│   │   │   ├── ./shared/assets/template_repos/benchmark_generator/plan.md # Markdown documentation or specification.
│   │   │   └── ./shared/assets/template_repos/benchmark_generator/result.py # Python source code.
│   │   └── ./shared/assets/template_repos/engineer
│   │   └── ./shared/assets/template_repos/engineer/result.py # Python source code.
│   ├── ./shared/cli
│   │   └── ./shared/cli/benchmark.py # Python source code.
│   ├── ./shared/cots # Commercial Off-The-Shelf parts logic.
│   │   ├── ./shared/cots/agent.py # Python source code.
│   │   ├── ./shared/cots/database
│   │   │   ├── ./shared/cots/database/__init__.py # Package initialization file.
│   │   │   ├── ./shared/cots/database/init.py # Python source code.
│   │   │   └── ./shared/cots/database/models.py # Python source code.
│   │   ├── ./shared/cots/indexer.py # Python source code.
│   │   ├── ./shared/cots/__init__.py # Package initialization file.
│   │   ├── ./shared/cots/models.py # Python source code.
│   │   └── ./shared/cots/runtime.py # Python source code.
│   ├── ./shared/enums.py # Python source code.
│   ├── ./shared/git_utils.py # Python source code.
│   ├── ./shared/__init__.py # Package initialization file.
│   ├── ./shared/logging.py # Python source code.
│   ├── ./shared/models
│   │   └── ./shared/models/observability.py # Python source code.
│   ├── ./shared/observability
│   │   ├── ./shared/observability/__init__.py # Package initialization file.
│   │   ├── ./shared/observability/logging.py # Python source code.
│   │   ├── ./shared/observability/persistence.py # Python source code.
│   │   ├── ./shared/observability/storage.py # Python source code.
│   │   └── ./shared/observability/tracing.py # Python source code.
│   ├── ./shared/ops
│   │   ├── ./shared/ops/backup.py # Python source code.
│   │   ├── ./shared/ops/__init__.py # Package initialization file.
│   │   └── ./shared/ops/workflows.py # Python source code.
│   └── ./shared/type_checking.py # Python source code.
├── ./test_dir
│   └── ./test_dir/test.mp4
├── ./test_output.txt
├── ./tests # Test suite (unit, integration, e2e).
│   ├── ./tests/agent # Test file.
│   │   ├── ./tests/agent/__init__.py # Package initialization file.
│   │   ├── ./tests/agent/test_architect.py # Test file.
│   │   ├── ./tests/agent/test_critic.py # Test file.
│   │   ├── ./tests/agent/test_e2e.py # Test file.
│   │   ├── ./tests/agent/test_engineer.py # Test file.
│   │   ├── ./tests/agent/test_graph.py # Test file.
│   │   ├── ./tests/agent/test_prompt_manager.py # Test file.
│   │   ├── ./tests/agent/test_sidecar.py # Test file.
│   │   └── ./tests/agent/test_state.py # Test file.
│   ├── ./tests/controller # Test file.
│   │   ├── ./tests/controller/test_agent_graph.py # Test file.
│   │   ├── ./tests/controller/test_clients_worker.py # Test file.
│   │   └── ./tests/controller/test_prompts.py # Test file.
│   ├── ./tests/cots # Test file.
│   │   ├── ./tests/cots/test_indexer.py # Test file.
│   │   ├── ./tests/cots/test_search_agent.py # Test file.
│   │   └── ./tests/cots/test_search.py # Test file.
│   ├── ./tests/e2e # Test file.
│   │   ├── ./tests/e2e/test_benchmark_generation.py # Test file.
│   │   └── ./tests/e2e/test_simulation_engine.py # Test file.
│   ├── ./tests/generators # Test file.
│   │   ├── ./tests/generators/test_coder.py # Test file.
│   │   ├── ./tests/generators/test_graph.py # Test file.
│   │   └── ./tests/generators/test_planner_prompt.py # Test file.
│   ├── ./tests/integration # Test file.
│   │   ├── ./tests/integration/test_agent_real_llm.py # Test file.
│   │   ├── ./tests/integration/test_simulation_concurrency.py # Test file.
│   │   └── ./tests/integration/test_worker_concurrency.py # Test file.
│   ├── ./tests/observability # Test file.
│   │   ├── ./tests/observability/test_logging.py # Test file.
│   │   ├── ./tests/observability/test_persistence.py # Test file.
│   │   ├── ./tests/observability/test_storage.py # Test file.
│   │   └── ./tests/observability/test_tracing.py # Test file.
│   ├── ./tests/ops # Test file.
│   │   └── ./tests/ops/test_backup.py # Test file.
│   ├── ./tests/test_api_fuzzing.py # Test file.
│   ├── ./tests/test_controller_api_extended.py # Test file.
│   ├── ./tests/test_controller_persistence.py # Test file.
│   ├── ./tests/test_controller_tasks.py # Test file.
│   ├── ./tests/test_cots_foundation.py # Test file.
│   ├── ./tests/test_dashboard_api.py # Test file.
│   ├── ./tests/test_env_config.py # Test file.
│   ├── ./tests/test_episodes_api.py # Test file.
│   ├── ./tests/test_integration_docker.py # Test file.
│   ├── ./tests/test_interrupt.py # Test file.
│   ├── ./tests/test_streaming_assets.py # Test file.
│   ├── ./tests/test_worker_api.py # Test file.
│   ├── ./tests/test_worker_core.py # Test file.
│   ├── ./tests/test_wp02.py # Test file.
│   ├── ./tests/test_wp04_controller.py # Test file.
│   ├── ./tests/test_wp06_temporal.py # Test file.
│   ├── ./tests/test_wp07_observability.py # Test file.
│   ├── ./tests/workbenches # Test file.
│   │   ├── ./tests/workbenches/test_cnc.py # Test file.
│   │   ├── ./tests/workbenches/test_config.py # Test file.
│   │   ├── ./tests/workbenches/test_facade.py # Test file.
│   │   ├── ./tests/workbenches/test_im.py # Test file.
│   │   └── ./tests/workbenches/test_print_3d.py # Test file.
│   └── ./tests/worker # Test file.
│   ├── ./tests/worker/simulation # Test file.
│   │   ├── ./tests/worker/simulation/test_builder.py # Test file.
│   │   └── ./tests/worker/simulation/test_loop.py # Test file.
│   ├── ./tests/worker/test_benchmark_tools.py # Test file.
│   ├── ./tests/worker/test_filesystem.py # Test file.
│   ├── ./tests/worker/test_model_integration.py # Test file.
│   └── ./tests/worker/test_utils.py # Test file.
├── ./test_utils_manual.py # Python source code.
├── ./uv.lock # Dependency lock file.
├── ./worker # The 'Muscle': Executes code, simulations, and utilizes heavy tools.
│   ├── ./worker/agent_files # Template files for agents (journal, script, todo).
│   │   ├── ./worker/agent_files/journal.md # Markdown documentation or specification.
│   │   ├── ./worker/agent_files/script.py # Python source code.
│   │   └── ./worker/agent_files/todo.md # Markdown documentation or specification.
│   ├── ./worker/api # Worker API routes.
│   │   ├── ./worker/api/__init__.py # Package initialization file.
│   │   ├── ./worker/api/routes.py # Python source code.
│   │   └── ./worker/api/schema.py # Python source code.
│   ├── ./worker/app.py # Python source code.
│   ├── ./worker/Dockerfile
│   ├── ./worker/filesystem # Sandboxed filesystem implementation.
│   │   ├── ./worker/filesystem/backend.py # Python source code.
│   │   ├── ./worker/filesystem/db.py # Python source code.
│   │   ├── ./worker/filesystem/__init__.py # Package initialization file.
│   │   ├── ./worker/filesystem/router.py # Python source code.
│   │   └── ./worker/filesystem/watchdog.py # Python source code.
│   ├── ./worker/generators # Logic for code generation.
│   │   └── ./worker/generators/__init__.py # Package initialization file.
│   ├── ./worker/__init__.py # Package initialization file.
│   ├── ./worker/reviews
│   │   └── ./worker/reviews/__init__.py # Package initialization file.
│   ├── ./worker/runtime
│   │   ├── ./worker/runtime/executor.py # Python source code.
│   │   └── ./worker/runtime/__init__.py # Package initialization file.
│   ├── ./worker/simulation # MuJoCo simulation loop and rendering.
│   │   ├── ./worker/simulation/builder.py # Python source code.
│   │   ├── ./worker/simulation/__init__.py # Package initialization file.
│   │   ├── ./worker/simulation/loop.py # Python source code.
│   │   └── ./worker/simulation/renderer.py # Python source code.
│   ├── ./worker/skills # Skill management and synchronization.
│   │   ├── ./worker/skills/__init__.py # Package initialization file.
│   │   └── ./worker/skills/sync.py # Python source code.
│   ├── ./worker/utils # Utility functions accessible to agents.
│   │   ├── ./worker/utils/dfm.py # Python source code.
│   │   ├── ./worker/utils/docs.py # Python source code.
│   │   ├── ./worker/utils/filesystem.py # Python source code.
│   │   ├── ./worker/utils/git.py # Python source code.
│   │   ├── ./worker/utils/handover.py # Python source code.
│   │   ├── ./worker/utils/__init__.py # Package initialization file.
│   │   ├── ./worker/utils/rendering.py # Python source code.
│   │   ├── ./worker/utils/storage.py # Python source code.
│   │   └── ./worker/utils/validation.py # Python source code.
│   ├── ./worker/verify_dependencies.py # Python source code.
│   └── ./worker/workbenches # Manufacturing constraints and pricing logic (CNC, IM, 3D Print).
│   ├── ./worker/workbenches/analysis_utils.py # Python source code.
│   ├── ./worker/workbenches/base.py # Python source code.
│   ├── ./worker/workbenches/cnc.py # Python source code.
│   ├── ./worker/workbenches/config.py # Python source code.
│   ├── ./worker/workbenches/__init__.py # Package initialization file.
│   ├── ./worker/workbenches/injection_molding.py # Python source code.
│   ├── ./worker/workbenches/manufacturing_config.yaml # YAML configuration.
│   ├── ./worker/workbenches/models.py # Python source code.
│   └── ./worker/workbenches/print_3d.py # Python source code.
└── ./worker_openapi.json # OpenAPI specification for the Worker API.

106 directories, 460 files
