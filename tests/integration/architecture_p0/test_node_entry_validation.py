import os
import uuid
from pathlib import Path

import httpx
import pytest
import yaml

from controller.agent.node_entry_validation import (
    validate_seeded_workspace_handoff_artifacts,
)
from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkConfirmResponse,
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeCreateResponse,
    EpisodeResponse,
)
from controller.clients.worker import WorkerClient
from shared.agents.config import AgentsConfig, DraftingMode
from shared.enums import AgentName, EntryFailureDisposition, EpisodeStatus, TraceType
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    CostTotals,
    DraftingCallout,
    DraftingDimension,
    DraftingSheet,
    DraftingView,
    EntryValidationContext,
    MovedObject,
    ObjectivesSection,
    PartConfig,
    PhysicsConfig,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import ReviewManifest
from tests.integration.agent.helpers import (
    seed_benchmark_assembly_definition,
    wait_for_benchmark_state,
    wait_for_episode_terminal,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
AGENTS_CONFIG_PATH = Path("config/agents_config.yaml")

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
REPO_MANUFACTURING_CONFIG = Path(
    "worker_heavy/workbenches/manufacturing_config.yaml"
).read_text(encoding="utf-8")
pytestmark = pytest.mark.xdist_group(name="physics_sims")


def _default_benchmark_parts() -> list[dict[str, object]]:
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {"fixed": True, "material_id": "aluminum_6061"},
        }
    ]


async def _poll_engineer_episode(
    client: httpx.AsyncClient,
    episode_id: str,
    *,
    terminal_statuses: set[EpisodeStatus],
    max_attempts: int = 90,
) -> EpisodeResponse:
    return EpisodeResponse.model_validate(
        await wait_for_episode_terminal(
            client,
            episode_id,
            timeout_s=float(max_attempts),
            terminal_statuses=terminal_statuses,
        )
    )


async def _poll_benchmark_session(
    client: httpx.AsyncClient,
    session_id: str,
    *,
    terminal_statuses: set[EpisodeStatus],
    max_attempts: int = 90,
) -> EpisodeResponse:
    return EpisodeResponse.model_validate(
        await wait_for_benchmark_state(
            client,
            session_id,
            timeout_s=float(max_attempts),
            terminal_statuses=terminal_statuses,
        )
    )


def _entry_validation_from_episode(episode: EpisodeResponse) -> EntryValidationContext:
    assert episode.metadata_vars is not None
    additional_info = episode.metadata_vars.additional_info or {}
    raw = additional_info.get("entry_validation")
    assert isinstance(raw, dict), "Missing metadata.additional_info.entry_validation"
    return EntryValidationContext.model_validate(raw)


def _node_start_traces(episode: EpisodeResponse, node_name: str) -> list[str]:
    return [
        trace.content or ""
        for trace in (episode.traces or [])
        if trace.trace_type == TraceType.LOG
        and trace.name == node_name
        and "Starting task phase" in (trace.content or "")
    ]


def _agents_config_with_technical_drawing_modes(
    *,
    engineer_mode: DraftingMode = DraftingMode.OFF,
    benchmark_mode: DraftingMode = DraftingMode.OFF,
) -> AgentsConfig:
    data = yaml.safe_load(AGENTS_CONFIG_PATH.read_text(encoding="utf-8")) or {}
    agents = data.setdefault("agents", {})
    engineer_agent = agents.setdefault("engineer_planner", {})
    engineer_agent["technical_drawing_mode"] = engineer_mode
    benchmark_agent = agents.setdefault("benchmark_planner", {})
    benchmark_agent["technical_drawing_mode"] = benchmark_mode
    return AgentsConfig.model_validate(data)


def _drafting_validation_payloads(
    *,
    assembly_part_name: str,
    drafting_target: str,
) -> tuple[dict[str, object], dict[str, object]]:
    benchmark_definition = BenchmarkDefinition(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="sphere",
            material_id="aluminum_6061",
            start_position=(0.0, 0.0, 10.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
        benchmark_parts=_default_benchmark_parts(),
    )
    assembly_definition = AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            planner_target_max_unit_cost_usd=90.0,
            planner_target_max_weight_g=900.0,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[
            PartConfig(name=assembly_part_name, config=AssemblyPartConfig())
        ],
        totals=CostTotals(
            estimated_unit_cost_usd=0.0,
            estimated_weight_g=0.0,
            estimate_confidence="high",
        ),
        drafting=DraftingSheet(
            sheet_id="sheet-1",
            title="Broken Drafting",
            views=[
                DraftingView(
                    view_id="front",
                    target=drafting_target,
                    projection="front",
                    datums=["A"],
                    dimensions=[
                        DraftingDimension(
                            dimension_id="width",
                            kind="linear",
                            target=drafting_target,
                            value=10.0,
                            binding=True,
                        )
                    ],
                    callouts=[
                        DraftingCallout(
                            callout_id="1",
                            label="Drafting target",
                            target=drafting_target,
                        )
                    ],
                )
            ],
        ),
    )
    return (
        benchmark_definition.model_dump(mode="json", by_alias=True, exclude_none=True),
        assembly_definition.model_dump(mode="json", by_alias=True, exclude_none=True),
    )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_184_seeded_workspace_rejects_mismatched_benchmark_caps():
    """
    INT-184: Seeded planner entry must reject copied benchmark caps that diverge
    from benchmark_definition.yaml.
    """
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(
                    min=(10.0, 10.0, 10.0),
                    max=(20.0, 20.0, 20.0),
                ),
                forbid_zones=[],
                build_zone=BoundingBox(
                    min=(-50.0, -50.0, 0.0),
                    max=(50.0, 50.0, 100.0),
                ),
            ),
            benchmark_parts=[
                {
                    "part_id": "environment_fixture",
                    "label": "environment_fixture",
                    "metadata": {
                        "fixed": True,
                        "material_id": "aluminum_6061",
                    },
                }
            ],
            simulation_bounds=BoundingBox(
                min=(-100.0, -100.0, 0.0),
                max=(100.0, 100.0, 100.0),
            ),
            moved_object=MovedObject(
                label="ball",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 50.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=54.0, max_weight_g=950.0),
        )
        assembly_definition = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=95.0,
                benchmark_max_weight_g=1800.0,
                planner_target_max_unit_cost_usd=90.0,
                planner_target_max_weight_g=1700.0,
            ),
            manufactured_parts=[],
            cots_parts=[],
            final_assembly=[],
            totals=CostTotals(
                estimated_unit_cost_usd=0.0,
                estimated_weight_g=0.0,
                estimate_confidence="high",
            ),
        )

        await worker.upload_file(
            "benchmark_definition.yaml",
            yaml.safe_dump(
                benchmark_definition.model_dump(mode="json", by_alias=True),
                sort_keys=False,
            ).encode("utf-8"),
            bypass_agent_permissions=True,
        )
        await worker.upload_file(
            "assembly_definition.yaml",
            yaml.safe_dump(
                assembly_definition.model_dump(mode="json", by_alias=True),
                sort_keys=False,
            ).encode("utf-8"),
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.ENGINEER_PLAN_REVIEWER,
        )
    finally:
        await worker.aclose()

    assert errors, "Expected mismatched benchmark caps to fail validation."
    assert any(error.artifact_path == "assembly_definition.yaml" for error in errors), (
        errors
    )
    assert any(
        "benchmark_max_unit_cost_usd" in error.message
        and "benchmark_definition.constraints.max_unit_cost" in error.message
        for error in errors
    ), errors
    assert any(
        "benchmark_max_weight_g" in error.message
        and "benchmark_definition.constraints.max_weight_g" in error.message
        for error in errors
    ), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "technical_drawing_mode",
    [
        pytest.param(DraftingMode.MINIMAL, id="minimal"),
        pytest.param(DraftingMode.FULL, id="full"),
    ],
)
async def test_int_184_seeded_workspace_requires_drafting_when_mode_enabled(
    monkeypatch: pytest.MonkeyPatch,
    technical_drawing_mode: DraftingMode,
):
    """
    INT-184: Seeded engineer-plan-reviewer entry must require drafting when the
    planner drafting mode is enabled.
    """
    drafting_config = _agents_config_with_technical_drawing_modes(
        engineer_mode=technical_drawing_mode,
    )
    monkeypatch.setattr(
        "worker_heavy.utils.file_validation.load_agents_config",
        lambda: drafting_config,
    )
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        assembly_definition = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                planner_target_max_unit_cost_usd=90.0,
                planner_target_max_weight_g=900.0,
            ),
            manufactured_parts=[],
            cots_parts=[],
            final_assembly=[],
            totals=CostTotals(
                estimated_unit_cost_usd=0.0,
                estimated_weight_g=0.0,
                estimate_confidence="high",
            ),
        )
        try:
            await worker.upload_file(
                "plan.md",
                (
                    b"## 1. Solution Overview\n"
                    b"- Keep the mechanism simple.\n"
                    b"\n"
                    b"## 2. Parts List\n"
                    b"- One target part.\n"
                    b"\n"
                    b"## 3. Assembly Strategy\n"
                    b"1. Assemble directly into the build zone.\n"
                    b"\n"
                    b"## 4. Assumption Register\n"
                    b"- Assumption: The planner relies on source-backed inputs that must be traceable.\n"
                    b"\n"
                    b"## 5. Detailed Calculations\n"
                    b"| ID | Problem / Decision | Result | Impact |\n"
                    b"| -- | -- | -- | -- |\n"
                    b"| CALC-001 | Example calculation supporting the plan | `N/A` | Replace this placeholder with the actual derived limit. |\n"
                    b"\n### CALC-001: Example calculation supporting the plan\n"
                    b"\n#### Problem Statement\n"
                    b"\nThe plan needs a traceable calculation instead of a freeform claim.\n"
                    b"\n#### Assumptions\n"
                    b"\n- `ASSUMP-001`: The input values are taken from the benchmark or assembly definition.\n"
                    b"\n#### Derivation\n"
                    b"\n- Compute the binding quantity from the declared inputs.\n"
                    b"\n#### Worst-Case Check\n"
                    b"\n- The derived limit must hold under the worst-case allowed inputs.\n"
                    b"\n#### Result\n"
                    b"\n- The design remains valid only if the derived limit is respected.\n"
                    b"\n#### Design Impact\n"
                    b"\n- Update the design or inputs if the calculation changes.\n"
                    b"\n#### Cross-References\n"
                    b"\n- `plan.md#3-assembly-strategy`\n"
                    b"\n"
                    b"## 6. Critical Constraints / Operating Envelope\n"
                    b"- Constraint: The mechanism must remain inside the derived operating limits.\n"
                    b"\n"
                    b"## 7. Cost & Weight Budget\n"
                    b"- Stay within budget.\n"
                    b"\n"
                    b"## 8. Risk Assessment\n"
                    b"- Minimal risk.\n"
                ),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "todo.md",
                b"- [ ] Review the plan\n",
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_definition.yaml",
                yaml.safe_dump(
                    benchmark_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "assembly_definition.yaml",
                yaml.safe_dump(
                    assembly_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "manufacturing_config.yaml",
                REPO_MANUFACTURING_CONFIG.encode("utf-8"),
                bypass_agent_permissions=True,
            )

            errors = await validate_seeded_workspace_handoff_artifacts(
                worker_client=worker,
                target_node=AgentName.ENGINEER_PLAN_REVIEWER,
            )
        finally:
            await worker.aclose()

    finally:
        pass

    assert errors, "Expected drafting mode to require assembly_definition.drafting."
    assert any(error.artifact_path == "assembly_definition.yaml" for error in errors), (
        errors
    )
    assert any("drafting" in error.message.lower() for error in errors), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "technical_drawing_mode",
    [
        pytest.param(DraftingMode.MINIMAL, id="minimal"),
        pytest.param(DraftingMode.FULL, id="full"),
    ],
)
async def test_int_184_seeded_workspace_rejects_unknown_drafting_targets(
    monkeypatch: pytest.MonkeyPatch,
    technical_drawing_mode: DraftingMode,
):
    """
    INT-184: Seeded engineer-plan-reviewer entry must reject drafting claims
    that point at undeclared assembly targets.
    """
    drafting_config = _agents_config_with_technical_drawing_modes(
        engineer_mode=technical_drawing_mode,
    )
    monkeypatch.setattr(
        "worker_heavy.utils.file_validation.load_agents_config",
        lambda: drafting_config,
    )
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        assembly_definition = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                planner_target_max_unit_cost_usd=90.0,
                planner_target_max_weight_g=900.0,
            ),
            manufactured_parts=[],
            cots_parts=[],
            final_assembly=[PartConfig(name="target_box", config=AssemblyPartConfig())],
            totals=CostTotals(
                estimated_unit_cost_usd=0.0,
                estimated_weight_g=0.0,
                estimate_confidence="high",
            ),
            drafting=DraftingSheet(
                sheet_id="sheet-1",
                title="Broken Drafting",
                views=[
                    DraftingView(
                        view_id="front",
                        target="ghost_part",
                        projection="front",
                        datums=["A"],
                        dimensions=[
                            DraftingDimension(
                                dimension_id="width",
                                kind="linear",
                                target="ghost_part",
                                value=10.0,
                                binding=True,
                            )
                        ],
                        callouts=[
                            DraftingCallout(
                                callout_id="1",
                                label="Ghost part",
                                target="ghost_part",
                            )
                        ],
                    )
                ],
            ),
        )
        try:
            await worker.upload_file(
                "plan.md",
                (
                    b"## 1. Solution Overview\n"
                    b"- Keep the mechanism simple.\n"
                    b"\n"
                    b"## 2. Parts List\n"
                    b"- One target part.\n"
                    b"\n"
                    b"## 3. Assembly Strategy\n"
                    b"1. Assemble directly into the build zone.\n"
                    b"\n"
                    b"## 4. Assumption Register\n"
                    b"- Assumption: The planner relies on source-backed inputs that must be traceable.\n"
                    b"\n"
                    b"## 5. Detailed Calculations\n"
                    b"| ID | Problem / Decision | Result | Impact |\n"
                    b"| -- | -- | -- | -- |\n"
                    b"| CALC-001 | Example calculation supporting the plan | `N/A` | Replace this placeholder with the actual derived limit. |\n"
                    b"\n### CALC-001: Example calculation supporting the plan\n"
                    b"\n#### Problem Statement\n"
                    b"\nThe plan needs a traceable calculation instead of a freeform claim.\n"
                    b"\n#### Assumptions\n"
                    b"\n- `ASSUMP-001`: The input values are taken from the benchmark or assembly definition.\n"
                    b"\n#### Derivation\n"
                    b"\n- Compute the binding quantity from the declared inputs.\n"
                    b"\n#### Worst-Case Check\n"
                    b"\n- The derived limit must hold under the worst-case allowed inputs.\n"
                    b"\n#### Result\n"
                    b"\n- The design remains valid only if the derived limit is respected.\n"
                    b"\n#### Design Impact\n"
                    b"\n- Update the design or inputs if the calculation changes.\n"
                    b"\n#### Cross-References\n"
                    b"\n- `plan.md#3-assembly-strategy`\n"
                    b"\n"
                    b"## 6. Critical Constraints / Operating Envelope\n"
                    b"- Constraint: The mechanism must remain inside the derived operating limits.\n"
                    b"\n"
                    b"## 7. Cost & Weight Budget\n"
                    b"- Stay within budget.\n"
                    b"\n"
                    b"## 8. Risk Assessment\n"
                    b"- Minimal risk.\n"
                ),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "todo.md",
                b"- [ ] Review the plan\n",
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_definition.yaml",
                yaml.safe_dump(
                    benchmark_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "assembly_definition.yaml",
                yaml.safe_dump(
                    assembly_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "manufacturing_config.yaml",
                REPO_MANUFACTURING_CONFIG.encode("utf-8"),
                bypass_agent_permissions=True,
            )

            errors = await validate_seeded_workspace_handoff_artifacts(
                worker_client=worker,
                target_node=AgentName.ENGINEER_PLAN_REVIEWER,
            )
        finally:
            await worker.aclose()

    finally:
        pass

    assert errors, "Expected unknown drafting targets to fail validation."
    assert any(error.artifact_path == "assembly_definition.yaml" for error in errors), (
        errors
    )
    assert any(
        "not tied to a declared assembly part" in error.message for error in errors
    ), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "technical_drawing_mode",
    [
        pytest.param(DraftingMode.MINIMAL, id="minimal"),
        pytest.param(DraftingMode.FULL, id="full"),
    ],
)
async def test_int_184_seeded_benchmark_workspace_requires_drafting_when_mode_enabled(
    monkeypatch: pytest.MonkeyPatch,
    technical_drawing_mode: DraftingMode,
):
    """
    INT-184: Seeded benchmark-plan-reviewer entry must require drafting when the
    benchmark planner drafting mode is enabled.
    """
    drafting_config = _agents_config_with_technical_drawing_modes(
        benchmark_mode=technical_drawing_mode,
    )
    monkeypatch.setattr(
        "worker_heavy.utils.file_validation.load_agents_config",
        lambda: drafting_config,
    )
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        assembly_definition = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                planner_target_max_unit_cost_usd=90.0,
                planner_target_max_weight_g=900.0,
            ),
            manufactured_parts=[],
            cots_parts=[],
            final_assembly=[],
            totals=CostTotals(
                estimated_unit_cost_usd=0.0,
                estimated_weight_g=0.0,
                estimate_confidence="high",
            ),
        )
        try:
            await worker.upload_file(
                "plan.md",
                (
                    b"## 1. Solution Overview\n"
                    b"- Keep the mechanism simple.\n"
                    b"\n"
                    b"## 2. Parts List\n"
                    b"- One target part.\n"
                    b"\n"
                    b"## 3. Assembly Strategy\n"
                    b"1. Assemble directly into the build zone.\n"
                    b"\n"
                    b"## 4. Assumption Register\n"
                    b"- Assumption: The planner relies on source-backed inputs that must be traceable.\n"
                    b"\n"
                    b"## 5. Detailed Calculations\n"
                    b"| ID | Problem / Decision | Result | Impact |\n"
                    b"| -- | -- | -- | -- |\n"
                    b"| CALC-001 | Example calculation supporting the plan | `N/A` | Replace this placeholder with the actual derived limit. |\n"
                    b"\n### CALC-001: Example calculation supporting the plan\n"
                    b"\n#### Problem Statement\n"
                    b"\nThe plan needs a traceable calculation instead of a freeform claim.\n"
                    b"\n#### Assumptions\n"
                    b"\n- `ASSUMP-001`: The input values are taken from the benchmark or assembly definition.\n"
                    b"\n#### Derivation\n"
                    b"\n- Compute the binding quantity from the declared inputs.\n"
                    b"\n#### Worst-Case Check\n"
                    b"\n- The derived limit must hold under the worst-case allowed inputs.\n"
                    b"\n#### Result\n"
                    b"\n- The design remains valid only if the derived limit is respected.\n"
                    b"\n#### Design Impact\n"
                    b"\n- Update the design or inputs if the calculation changes.\n"
                    b"\n#### Cross-References\n"
                    b"\n- `plan.md#3-assembly-strategy`\n"
                    b"\n"
                    b"## 6. Critical Constraints / Operating Envelope\n"
                    b"- Constraint: The mechanism must remain inside the derived operating limits.\n"
                    b"\n"
                    b"## 7. Cost & Weight Budget\n"
                    b"- Stay within budget.\n"
                    b"\n"
                    b"## 8. Risk Assessment\n"
                    b"- Minimal risk.\n"
                ),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "todo.md",
                b"- [ ] Review the plan\n",
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_definition.yaml",
                yaml.safe_dump(
                    benchmark_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_assembly_definition.yaml",
                yaml.safe_dump(
                    assembly_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "manufacturing_config.yaml",
                REPO_MANUFACTURING_CONFIG.encode("utf-8"),
                bypass_agent_permissions=True,
            )

            errors = await validate_seeded_workspace_handoff_artifacts(
                worker_client=worker,
                target_node=AgentName.BENCHMARK_PLAN_REVIEWER,
            )
        finally:
            await worker.aclose()

    finally:
        pass

    assert errors, (
        "Expected drafting mode to require benchmark_assembly_definition.drafting."
    )
    assert any(
        error.artifact_path == "benchmark_assembly_definition.yaml" for error in errors
    ), errors
    assert any("drafting" in error.message.lower() for error in errors), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "technical_drawing_mode",
    [
        pytest.param(DraftingMode.MINIMAL, id="minimal"),
        pytest.param(DraftingMode.FULL, id="full"),
    ],
)
async def test_int_184_seeded_benchmark_workspace_rejects_unknown_drafting_targets(
    monkeypatch: pytest.MonkeyPatch,
    technical_drawing_mode: DraftingMode,
):
    """
    INT-184: Seeded benchmark-plan-reviewer entry must reject drafting claims
    that point at undeclared benchmark assembly targets.
    """
    drafting_config = _agents_config_with_technical_drawing_modes(
        benchmark_mode=technical_drawing_mode,
    )
    monkeypatch.setattr(
        "worker_heavy.utils.file_validation.load_agents_config",
        lambda: drafting_config,
    )
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        benchmark_assembly_definition = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                planner_target_max_unit_cost_usd=90.0,
                planner_target_max_weight_g=900.0,
            ),
            manufactured_parts=[],
            cots_parts=[],
            final_assembly=[
                PartConfig(name="environment_fixture", config=AssemblyPartConfig())
            ],
            totals=CostTotals(
                estimated_unit_cost_usd=0.0,
                estimated_weight_g=0.0,
                estimate_confidence="high",
            ),
            drafting=DraftingSheet(
                sheet_id="sheet-1",
                title="Broken Benchmark Drafting",
                views=[
                    DraftingView(
                        view_id="front",
                        target="ghost_fixture",
                        projection="front",
                        datums=["A"],
                        dimensions=[
                            DraftingDimension(
                                dimension_id="width",
                                kind="linear",
                                target="ghost_fixture",
                                value=10.0,
                                binding=True,
                            )
                        ],
                        callouts=[
                            DraftingCallout(
                                callout_id="1",
                                label="Ghost fixture",
                                target="ghost_fixture",
                            )
                        ],
                    )
                ],
            ),
        )
        try:
            await worker.upload_file(
                "plan.md",
                (
                    b"## 1. Learning Objective\n"
                    b"- Move the ball around the obstacle.\n"
                    b"\n"
                    b"## 2. Geometry\n"
                    b"- Keep the obstacle between the start and goal.\n"
                    b"\n"
                    b"## 3. Objectives\n"
                    b"- The ball must reach the goal without entering the forbid zone.\n"
                ),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "todo.md",
                b"- [ ] Review the plan\n",
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_definition.yaml",
                yaml.safe_dump(
                    benchmark_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_assembly_definition.yaml",
                yaml.safe_dump(
                    benchmark_assembly_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "manufacturing_config.yaml",
                REPO_MANUFACTURING_CONFIG.encode("utf-8"),
                bypass_agent_permissions=True,
            )

            errors = await validate_seeded_workspace_handoff_artifacts(
                worker_client=worker,
                target_node=AgentName.BENCHMARK_PLAN_REVIEWER,
            )
        finally:
            await worker.aclose()

    finally:
        pass

    assert errors, "Expected unknown drafting targets to fail validation."
    assert any(
        error.artifact_path == "benchmark_assembly_definition.yaml" for error in errors
    ), errors
    assert any(
        "not tied to a declared assembly part" in error.message for error in errors
    ), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "target_node, assembly_artifact, assembly_part_name, drafting_target, technical_drawing_mode",
    [
        pytest.param(
            AgentName.ENGINEER_PLAN_REVIEWER,
            "assembly_definition.yaml",
            "target_box",
            "target_box",
            DraftingMode.FULL,
            id="engineer",
        ),
        pytest.param(
            AgentName.BENCHMARK_PLAN_REVIEWER,
            "benchmark_assembly_definition.yaml",
            "environment_fixture",
            "environment_fixture",
            DraftingMode.FULL,
            id="benchmark",
        ),
    ],
)
@pytest.mark.parametrize(
    "drafting_case, expected_error",
    [
        pytest.param(
            "unsupported_view_projection",
            "projection 'isometric' is not supported",
            id="unsupported-view",
        ),
        pytest.param(
            "duplicate_datums",
            "must not contain duplicate datum identifiers",
            id="duplicate-datums",
        ),
        pytest.param(
            "unknown_callout_target",
            "not tied to a declared assembly part",
            id="unknown-callout-target",
        ),
        pytest.param(
            "unknown_dimension_target",
            "not tied to a declared assembly part",
            id="unknown-dimension-target",
        ),
    ],
)
async def test_int_184_seeded_workspace_rejects_invalid_drafting_contract(
    monkeypatch: pytest.MonkeyPatch,
    target_node: AgentName,
    assembly_artifact: str,
    assembly_part_name: str,
    drafting_target: str,
    technical_drawing_mode: DraftingMode,
    drafting_case: str,
    expected_error: str,
):
    """
    INT-184: Seeded workspace entries must fail closed on malformed drafting
    views, datums, callouts, and dimensions.
    """
    drafting_config = _agents_config_with_technical_drawing_modes(
        engineer_mode=technical_drawing_mode
        if target_node == AgentName.ENGINEER_PLAN_REVIEWER
        else DraftingMode.OFF,
        benchmark_mode=technical_drawing_mode
        if target_node == AgentName.BENCHMARK_PLAN_REVIEWER
        else DraftingMode.OFF,
    )
    monkeypatch.setattr(
        "worker_heavy.utils.file_validation.load_agents_config",
        lambda: drafting_config,
    )
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_payload, assembly_payload = _drafting_validation_payloads(
            assembly_part_name=assembly_part_name,
            drafting_target=drafting_target,
        )
        drafting_view = assembly_payload["drafting"]["views"][0]
        ghost_target = f"ghost_{assembly_part_name}"
        if drafting_case == "unsupported_view_projection":
            drafting_view["projection"] = "isometric"
        elif drafting_case == "duplicate_datums":
            drafting_view["datums"] = ["A", "A"]
        elif drafting_case == "unknown_callout_target":
            drafting_view["callouts"][0]["target"] = ghost_target
        elif drafting_case == "unknown_dimension_target":
            drafting_view["dimensions"][0]["target"] = ghost_target
        else:
            raise AssertionError(f"Unknown drafting case: {drafting_case}")

        await worker.upload_file(
            "plan.md",
            (
                "## 1. Solution Overview\n"
                "- Keep the mechanism simple.\n"
                "\n"
                "## 2. Parts List\n"
                f"- One {assembly_part_name}.\n"
                "\n"
                "## 3. Assembly Strategy\n"
                "1. Assemble directly into the build zone.\n"
                "\n"
                "## 4. Assumption Register\n"
                "- Assumption: The planner relies on source-backed inputs that must be traceable.\n"
                "\n"
                "## 5. Detailed Calculations\n"
                "| ID | Problem / Decision | Result | Impact |\n"
                "| -- | -- | -- | -- |\n"
                "| CALC-001 | Example calculation supporting the plan | \`N/A\` | Replace this placeholder with the actual derived limit. |\n"
                "\n### CALC-001: Example calculation supporting the plan\n"
                "\n#### Problem Statement\n"
                "\nThe plan needs a traceable calculation instead of a freeform claim.\n"
                "\n#### Assumptions\n"
                "\n- \`ASSUMP-001\`: The input values are taken from the benchmark or assembly definition.\n"
                "\n#### Derivation\n"
                "\n- Compute the binding quantity from the declared inputs.\n"
                "\n#### Worst-Case Check\n"
                "\n- The derived limit must hold under the worst-case allowed inputs.\n"
                "\n#### Result\n"
                "\n- The design remains valid only if the derived limit is respected.\n"
                "\n#### Design Impact\n"
                "\n- Update the design or inputs if the calculation changes.\n"
                "\n#### Cross-References\n"
                "\n- \`plan.md#3-assembly-strategy\`\n"
                "\n"
                "## 6. Critical Constraints / Operating Envelope\n"
                "- Constraint: The mechanism must remain inside the derived operating limits.\n"
                "\n"
                "## 7. Cost & Weight Budget\n"
                "- Stay within budget.\n"
                "\n"
                "## 8. Risk Assessment\n"
                "- Minimal risk.\n"
            ).encode(),
            bypass_agent_permissions=True,
        )
        await worker.upload_file(
            "todo.md",
            b"- [ ] Review the plan\n",
            bypass_agent_permissions=True,
        )
        await worker.upload_file(
            "benchmark_definition.yaml",
            yaml.safe_dump(benchmark_payload, sort_keys=False).encode("utf-8"),
            bypass_agent_permissions=True,
        )
        await worker.upload_file(
            assembly_artifact,
            yaml.safe_dump(assembly_payload, sort_keys=False).encode("utf-8"),
            bypass_agent_permissions=True,
        )
        await worker.upload_file(
            "manufacturing_config.yaml",
            REPO_MANUFACTURING_CONFIG.encode("utf-8"),
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=target_node,
        )
    finally:
        await worker.aclose()

    assert errors, "Expected malformed drafting content to fail validation."
    assert any(error.artifact_path == assembly_artifact for error in errors), errors
    assert any(expected_error in error.message for error in errors), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_190_seeded_benchmark_workspace_requires_benchmark_script():
    """
    INT-190: Benchmark-backed seeded workspaces must fail closed when the
    benchmark script source is missing.
    """
    session_id = f"INT-190-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        benchmark_definition = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        async with httpx.AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
            await worker.upload_file(
                "plan.md",
                (
                    b"## 1. Learning Objective\n"
                    b"- Move the ball around the obstacle.\n"
                    b"\n"
                    b"## 2. Geometry\n"
                    b"- Keep the obstacle between the start and goal.\n"
                    b"\n"
                    b"## 3. Objectives\n"
                    b"- The ball must reach the goal without entering the forbid zone.\n"
                ),
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "todo.md",
                b"- [ ] Review the plan\n",
                bypass_agent_permissions=True,
            )
            await worker.upload_file(
                "benchmark_definition.yaml",
                yaml.safe_dump(
                    benchmark_definition.model_dump(
                        mode="json", by_alias=True, exclude_none=True
                    ),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await seed_benchmark_assembly_definition(
                client,
                session_id,
                planner_target_max_unit_cost_usd=90.0,
                planner_target_max_weight_g=900.0,
            )
            await worker.execute_command("rm -f benchmark_script.py", timeout=10)

            errors = await validate_seeded_workspace_handoff_artifacts(
                worker_client=worker,
                target_node=AgentName.BENCHMARK_PLAN_REVIEWER,
            )
    finally:
        await worker.aclose()

    assert errors, (
        "Expected benchmark-backed seed validation to require benchmark_script.py."
    )
    assert any(error.artifact_path == "benchmark_script.py" for error in errors), errors
    assert any("benchmark_script.py" in error.message for error in errors), errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_184_seeded_benchmark_assembly_uses_benchmark_definition_caps_only():
    """
    INT-184: Seeded benchmark planner handoff should validate when benchmark caps
    come from benchmark_definition.yaml and are not duplicated in the benchmark
    assembly file.
    """
    session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        async with httpx.AsyncClient(timeout=300.0) as client:
            benchmark_definition = BenchmarkDefinition(
                objectives=ObjectivesSection(
                    goal_zone=BoundingBox(
                        min=(10.0, 10.0, 10.0),
                        max=(20.0, 20.0, 20.0),
                    ),
                    forbid_zones=[],
                    build_zone=BoundingBox(
                        min=(-50.0, -50.0, 0.0),
                        max=(50.0, 50.0, 100.0),
                    ),
                ),
                benchmark_parts=[
                    {
                        "part_id": "environment_fixture",
                        "label": "environment_fixture",
                        "metadata": {
                            "fixed": True,
                            "material_id": "aluminum_6061",
                        },
                    }
                ],
                simulation_bounds=BoundingBox(
                    min=(-100.0, -100.0, 0.0),
                    max=(100.0, 100.0, 100.0),
                ),
                moved_object=MovedObject(
                    label="ball",
                    shape="sphere",
                    material_id="aluminum_6061",
                    start_position=(0.0, 0.0, 50.0),
                    runtime_jitter=(0.0, 0.0, 0.0),
                ),
                constraints=Constraints(
                    estimated_solution_cost_usd=133.3333333333,
                    estimated_solution_weight_g=666.6666666667,
                    max_unit_cost=200.0,
                    max_weight_g=1000.0,
                ),
            )

            await worker.upload_file(
                "benchmark_definition.yaml",
                yaml.safe_dump(
                    benchmark_definition.model_dump(mode="json", by_alias=True),
                    sort_keys=False,
                ).encode("utf-8"),
                bypass_agent_permissions=True,
            )
            await seed_benchmark_assembly_definition(
                client,
                session_id,
                benchmark_max_unit_cost_usd=200.0,
                benchmark_max_weight_g=1000.0,
            )
        content = await worker.read_file(
            "benchmark_assembly_definition.yaml",
            bypass_agent_permissions=True,
        )
        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.BENCHMARK_PLANNER,
        )
    finally:
        await worker.aclose()

    assert "benchmark_max_unit_cost_usd" not in content
    assert "benchmark_max_weight_g" not in content
    assert not errors, errors


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "node_entry_validation_rejected",
        "execution reviewer entry blocked",
        "reviewer handover",
    ]
)
@pytest.mark.asyncio
async def test_int_184_engineer_fail_fast_and_skip_target_node():
    """
    INT-184: Engineer path entry validation must fail-fast, skip target node execution,
    and persist structured metadata including reroute_target for non-integration parity.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-184 engineer node-entry fail-fast contract.",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_EXECUTION_REVIEWER,
            start_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _poll_engineer_episode(
            client,
            str(run.episode_id),
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.COMPLETED},
        )
        assert episode.status == EpisodeStatus.FAILED

        entry = _entry_validation_from_episode(episode)
        assert entry.node == AgentName.ENGINEER_EXECUTION_REVIEWER
        assert entry.disposition == EntryFailureDisposition.FAIL_FAST
        assert entry.reason_code == "reviewer_entry_blocked"
        assert entry.reroute_target == AgentName.ENGINEER_CODER
        assert entry.errors
        assert all(err.code and err.message and err.source for err in entry.errors)

        additional_info = episode.metadata_vars.additional_info or {}
        assert additional_info.get("entry_validation_terminal") is True
        assert not _node_start_traces(
            episode, AgentName.ENGINEER_EXECUTION_REVIEWER.value
        )

        failure_logs = [
            log
            for log in (episode.metadata_vars.validation_logs or [])
            if log.startswith("ENTRY_VALIDATION_FAILED[")
        ]
        assert len(failure_logs) == 1

        failure_traces = [
            t
            for t in (episode.traces or [])
            if t.name == "node_entry_validation_failed"
            and t.trace_type in {TraceType.EVENT, TraceType.ERROR}
        ]
        assert failure_traces, "Expected node_entry_validation_failed traces."


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "node_entry_validation_rejected",
        "benchmark_assembly_definition.yaml",
        "missing_artifact",
    ]
)
@pytest.mark.asyncio
async def test_int_184_engineer_planner_requires_benchmark_assembly_definition():
    """
    INT-184: Engineer planner entry must fail closed when benchmark_assembly_definition.yaml is absent.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-184 engineer planner requires benchmark assembly context.",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_PLANNER,
            start_node=AgentName.ENGINEER_PLANNER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _poll_engineer_episode(
            client,
            str(run.episode_id),
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.COMPLETED},
        )
        assert episode.status == EpisodeStatus.FAILED

        entry = _entry_validation_from_episode(episode)
        assert entry.node == AgentName.ENGINEER_PLANNER
        assert entry.disposition == EntryFailureDisposition.FAIL_FAST
        assert entry.reason_code == "missing_artifact"
        assert entry.reroute_target is None
        assert any(
            error.artifact_path == "benchmark_assembly_definition.yaml"
            for error in entry.errors
        )
        assert any(
            "benchmark_assembly_definition.yaml" in error.message
            for error in entry.errors
        )
        assert not _node_start_traces(episode, AgentName.ENGINEER_PLANNER.value)


@pytest.mark.integration_p0
@pytest.mark.xdist_group(name="physics_sims")
@pytest.mark.allow_backend_errors(
    regexes=[
        "approved_benchmark_bundle_validation_failed",
        "benchmark handoff failed",
        "benchmark_review_manifest.json",
    ]
)
@pytest.mark.asyncio
async def test_int_184_engineer_planner_rejects_stale_benchmark_bundle():
    """
    INT-184: Engineer planner entry must fail closed when the approved benchmark
    bundle is revision-mismatched before the run starts.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        benchmark_request = BenchmarkGenerateRequest(
            prompt="INT-005: Create a benchmark about stacking blocks.",
            backend=SimulatorBackendType.GENESIS,
        )
        benchmark_resp = await client.post(
            f"{CONTROLLER_URL}/api/benchmark/generate",
            json=benchmark_request.model_dump(mode="json"),
        )
        assert benchmark_resp.status_code in [200, 202], benchmark_resp.text
        benchmark_run = BenchmarkGenerateResponse.model_validate(benchmark_resp.json())
        benchmark_session_id = str(benchmark_run.session_id)

        benchmark_episode = await _poll_benchmark_session(
            client,
            benchmark_session_id,
            terminal_statuses={EpisodeStatus.PLANNED, EpisodeStatus.COMPLETED},
        )
        if benchmark_episode.status == EpisodeStatus.PLANNED:
            confirm_resp = await client.post(
                f"{CONTROLLER_URL}/api/benchmark/{benchmark_session_id}/confirm",
                json=ConfirmRequest(comment="Approve benchmark").model_dump(
                    mode="json"
                ),
            )
            assert confirm_resp.status_code in [200, 202], confirm_resp.text
            benchmark_episode = await _poll_benchmark_session(
                client,
                benchmark_session_id,
                terminal_statuses={EpisodeStatus.COMPLETED},
            )

        assert benchmark_episode.status == EpisodeStatus.COMPLETED

        benchmark_worker = WorkerClient(
            base_url=WORKER_LIGHT_URL,
            session_id=benchmark_session_id,
        )
        try:
            manifest_path = ".manifests/benchmark_review_manifest.json"
            manifest_raw = await benchmark_worker.read_file(
                manifest_path, bypass_agent_permissions=True
            )
            manifest = ReviewManifest.model_validate_json(manifest_raw)
            tampered_manifest = manifest.model_copy(
                update={"revision": "0000000000000000000000000000000000000000"}
            )
            await benchmark_worker.write_file(
                manifest_path,
                tampered_manifest.model_dump_json(indent=2),
                overwrite=True,
                bypass_agent_permissions=True,
            )
        finally:
            await benchmark_worker.aclose()

        engineer_session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-184 engineer planner stale benchmark bundle rejection.",
            session_id=engineer_session_id,
            agent_name=AgentName.ENGINEER_PLANNER,
            start_node=AgentName.ENGINEER_PLANNER,
            metadata_vars={"benchmark_id": benchmark_session_id},
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _poll_engineer_episode(
            client,
            str(run.episode_id),
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.COMPLETED},
        )
        assert episode.status == EpisodeStatus.FAILED
        assert episode.metadata_vars is not None
        assert episode.metadata_vars.benchmark_id == benchmark_session_id
        assert episode.metadata_vars.terminal_reason == "HANDOFF_INVARIANT_VIOLATION"
        assert episode.metadata_vars.failure_class == "AGENT_SEMANTIC_FAILURE"
        assert any(
            "approved benchmark bundle validation failed" in log.lower()
            or "benchmark handoff failed" in log.lower()
            for log in (episode.metadata_vars.validation_logs or [])
        )
        assert not _node_start_traces(episode, AgentName.ENGINEER_PLANNER.value)
        assert any(
            trace.name == "benchmark_handoff_validation_failed"
            for trace in (episode.traces or [])
        )


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "continue_generation_invalid_episode_status",
    ]
)
@pytest.mark.asyncio
async def test_int_184_benchmark_fail_fast_state_invalid_confirm_path():
    """
    INT-184: Benchmark path invalid entry must fail-fast with structured metadata
    and no benchmark_coder node execution evidence.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        create_req = AgentRunRequest(
            task="INT-184 benchmark state-invalid confirm path.",
            session_id=f"INT-184-benchmark-{uuid.uuid4().hex[:8]}",
        )
        create_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json=create_req.model_dump(mode="json"),
        )
        assert create_resp.status_code == 201, create_resp.text
        created = EpisodeCreateResponse.model_validate(create_resp.json())
        session_id = str(created.episode_id)

        confirm_resp = await client.post(
            f"{CONTROLLER_URL}/api/benchmark/{session_id}/confirm",
            json={"comment": "INT-184 invalid status", "additional_turns": 0},
        )
        assert confirm_resp.status_code == 200, confirm_resp.text
        BenchmarkConfirmResponse.model_validate(confirm_resp.json())

        session = await _poll_benchmark_session(
            client,
            session_id,
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.PLANNED},
        )
        assert session.status == EpisodeStatus.FAILED

        entry = _entry_validation_from_episode(session)
        assert entry.node == AgentName.BENCHMARK_CODER
        assert entry.disposition == EntryFailureDisposition.FAIL_FAST
        assert entry.reason_code == "state_invalid"
        assert entry.reroute_target is None
        assert entry.errors

        assert not _node_start_traces(session, AgentName.BENCHMARK_CODER.value)
        assert any(
            "continue_generation_session requires PLANNED episode status" in log
            for log in (session.metadata_vars.validation_logs or [])
        )
