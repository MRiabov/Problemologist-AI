import asyncio
import uuid

import httpx
import pytest
import yaml

from controller.api.schemas import AgentRunRequest, EpisodeCreateResponse
from shared.enums import AgentName
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    CostTotals,
    MovedObject,
    ObjectivesSection,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    WriteFileRequest,
)

CONTROLLER_URL = "http://127.0.0.1:18000"
WORKER_LIGHT_URL = "http://127.0.0.1:18001"
WORKER_HEAVY_URL = "http://127.0.0.1:18002"

_BUSY_SLEEP_SECONDS = 8


def _benchmark_definition() -> BenchmarkDefinition:
    return BenchmarkDefinition(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(-2.0, -2.0, 0.0), max=(2.0, 2.0, 10.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
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
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="sphere",
            material_id="aluminum_6061",
            start_position=(0.0, 0.0, 4.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
    )


def _benchmark_assembly_definition() -> AssemblyDefinition:
    return AssemblyDefinition(
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=100.0,
            benchmark_max_weight_g=1000.0,
            planner_target_max_unit_cost_usd=90.0,
            planner_target_max_weight_g=900.0,
        ),
        totals=CostTotals(
            estimated_unit_cost_usd=10.0,
            estimated_weight_g=100.0,
            estimate_confidence="high",
        ),
    )


def _plan_markdown() -> str:
    return """## 1. Solution Overview
A valid solution overview.

## 2. Parts List
| Part | Qty |
|------|-----|
| Box  | 1   |

## 3. Assembly Strategy
1. Step one.

## 4. Cost & Weight Budget
- Cost: $10
- Weight: 100g

## 5. Risk Assessment
- Risk: Low
"""


def _todo_markdown() -> str:
    return "- [x] Step 1\n- [-] Step 2\n"


def _sleeping_script() -> str:
    return """
from build123d import Location, Sphere
from shared.models.schemas import PartMetadata


def build():
    part = Sphere(1.0).move(Location((0, 0, 4)))
    part.label = "target_box"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return part
"""


async def _write_session_workspace(
    client: httpx.AsyncClient, session_id: str, script_path: str
) -> None:
    headers = {"X-Session-ID": session_id}
    script_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path=script_path,
            content=_sleeping_script(),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert script_resp.status_code == 200, script_resp.text

    objectives_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="benchmark_definition.yaml",
            content=yaml.safe_dump(
                _benchmark_definition().model_dump(mode="json"), sort_keys=False
            ),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert objectives_resp.status_code == 200, objectives_resp.text

    plan_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="plan.md",
            content=_plan_markdown(),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert plan_resp.status_code == 200, plan_resp.text

    todo_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="todo.md",
            content=_todo_markdown(),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert todo_resp.status_code == 200, todo_resp.text

    assembly_resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="benchmark_assembly_definition.yaml",
            content=yaml.safe_dump(
                _benchmark_assembly_definition().model_dump(mode="json"),
                sort_keys=False,
            ),
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert assembly_resp.status_code == 200, assembly_resp.text


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_192_controller_script_tools_validate_waits_through_temporal_queue():
    """
    INT-192: controller script-tools validate/simulate/submit must wait through the
    Temporal-backed heavy path instead of leaking a raw worker-heavy busy response.
    """
    busy_session_id = f"INT-192-BUSY-{uuid.uuid4().hex[:8]}"
    tool_session_id = f"INT-192-TOOLS-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=600.0) as client:
        await _write_session_workspace(client, busy_session_id, "busy_script.py")
        await _write_session_workspace(client, tool_session_id, "script.py")

        create_episode_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json=AgentRunRequest(
                task="INT-192 controller script-tools proxy",
                session_id=tool_session_id,
                agent_name=AgentName.BENCHMARK_CODER,
            ).model_dump(mode="json"),
            timeout=120.0,
        )
        assert create_episode_resp.status_code == 201, create_episode_resp.text
        EpisodeCreateResponse.model_validate(create_episode_resp.json())

        busy_task = asyncio.create_task(
            client.post(
                f"{WORKER_HEAVY_URL}/benchmark/validate",
                json=BenchmarkToolRequest(
                    script_path="busy_script.py",
                    backend=SimulatorBackendType.MUJOCO,
                ).model_dump(mode="json"),
                headers={"X-Session-ID": busy_session_id},
                timeout=600.0,
            )
        )

        worker_busy = False
        for _ in range(80):
            ready_resp = await client.get(f"{WORKER_HEAVY_URL}/ready", timeout=5.0)
            if ready_resp.status_code != 200:
                worker_busy = True
                break
            await asyncio.sleep(0.2)

        assert worker_busy, "worker-heavy never reported busy during the control run"

        validate_resp = await client.post(
            f"{CONTROLLER_URL}/api/script-tools/validate",
            json={
                "script_path": "script.py",
                "agent_role": AgentName.BENCHMARK_CODER.value,
            },
            headers={"X-Session-ID": tool_session_id},
            timeout=600.0,
        )

        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message
        assert validate_data.artifacts is not None
        assert validate_data.artifacts.validation_results_json is not None

        simulate_resp = await client.post(
            f"{CONTROLLER_URL}/api/script-tools/simulate",
            json={
                "script_path": "script.py",
                "agent_role": AgentName.BENCHMARK_CODER.value,
            },
            headers={"X-Session-ID": tool_session_id},
            timeout=600.0,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success, simulate_data.message
        assert simulate_data.artifacts is not None
        assert simulate_data.artifacts.simulation_result_json is not None

        submit_resp = await client.post(
            f"{CONTROLLER_URL}/api/script-tools/submit",
            json={
                "script_path": "script.py",
                "agent_role": AgentName.BENCHMARK_CODER.value,
                "reviewer_stage": "benchmark_reviewer",
            },
            headers={"X-Session-ID": tool_session_id},
            timeout=600.0,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success, submit_data.message
        assert submit_data.artifacts is not None
        assert submit_data.artifacts.review_manifests_json

        assert "WORKER_BUSY" not in validate_resp.text
        assert "WORKER_BUSY" not in simulate_resp.text
        assert "WORKER_BUSY" not in submit_resp.text

        busy_resp = await busy_task
        assert busy_resp.status_code == 200, busy_resp.text
        busy_data = BenchmarkToolResponse.model_validate(busy_resp.json())
        assert busy_data.success, busy_data.message
