import asyncio
import json
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

pytestmark = pytest.mark.xdist_group(name="integration_shared_serial")

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
@pytest.mark.int_id("INT-192")
async def test_int_192_controller_script_tools_validate_waits_through_temporal_queue():
    """
    INT-192: controller script-tools validate/simulate/submit must wait through the
    Temporal-backed heavy path instead of leaking a raw worker-heavy busy response.
    """
    busy_session_id = f"INT-192-BUSY-{uuid.uuid4().hex[:8]}"
    tool_session_id = f"INT-192-TOOLS-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=1000.0) as client:
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
                timeout=1000.0,
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
            timeout=1000.0,
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
            timeout=1000.0,
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
            timeout=1000.0,
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


@pytest.mark.integration_p1
@pytest.mark.asyncio
@pytest.mark.int_id("INT-193")
async def test_int_193_controller_script_tools_verify_waits_through_temporal_queue():
    """
    INT-193: controller script-tools verify must wait through the Temporal-backed
    heavy path instead of leaking a raw worker-heavy busy response.
    """
    busy_session_id = f"INT-193-BUSY-{uuid.uuid4().hex[:8]}"
    tool_session_id = f"INT-193-TOOLS-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=1000.0) as client:
        await _write_session_workspace(client, busy_session_id, "busy_script.py")
        await _write_session_workspace(client, tool_session_id, "script.py")

        create_episode_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json=AgentRunRequest(
                task="INT-193 controller script-tools verify proxy",
                session_id=tool_session_id,
                agent_name=AgentName.BENCHMARK_CODER,
            ).model_dump(mode="json"),
            timeout=120.0,
        )
        assert create_episode_resp.status_code == 201, create_episode_resp.text
        EpisodeCreateResponse.model_validate(create_episode_resp.json())

        tool_validation_results = json.dumps(
            {
                "success": True,
                "message": "seeded",
                "timestamp": 0.0,
                "script_path": "script.py",
                "script_sha256": "1" * 64,
                "verification_result": None,
            }
        )
        tool_validation_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="validation_results.json",
                content=tool_validation_results,
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": tool_session_id},
            timeout=1000.0,
        )
        assert tool_validation_resp.status_code == 200, tool_validation_resp.text

        busy_validation_results = json.dumps(
            {
                "success": True,
                "message": "seeded",
                "timestamp": 0.0,
                "script_path": "busy_script.py",
                "script_sha256": "0" * 64,
                "verification_result": None,
            }
        )
        busy_validation_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="validation_results.json",
                content=busy_validation_results,
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": busy_session_id},
            timeout=1000.0,
        )
        assert busy_validation_resp.status_code == 200, busy_validation_resp.text

        busy_task = asyncio.create_task(
            client.post(
                f"{WORKER_HEAVY_URL}/benchmark/verify",
                json={
                    "script_path": "busy_script.py",
                    "backend": SimulatorBackendType.MUJOCO.value,
                    "smoke_test_mode": True,
                    "num_scenes": 5,
                    "duration": 10.0,
                    "jitter_range": [0.0, 0.0, 0.0],
                    "seed": 42,
                },
                headers={"X-Session-ID": busy_session_id},
                timeout=1000.0,
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

        verify_resp = await client.post(
            f"{CONTROLLER_URL}/api/script-tools/verify",
            json={
                "script_path": "script.py",
                "agent_role": AgentName.BENCHMARK_CODER.value,
                "backend": SimulatorBackendType.MUJOCO.value,
                "smoke_test_mode": True,
                "jitter_range": [0.0, 0.0, 0.0],
                "seed": 42,
            },
            headers={"X-Session-ID": tool_session_id},
            timeout=1000.0,
        )

        assert verify_resp.status_code == 200, verify_resp.text
        verify_data = BenchmarkToolResponse.model_validate(verify_resp.json())
        assert verify_data.success, verify_data.message
        assert verify_data.artifacts is not None
        assert verify_data.artifacts.verification_result is not None
        assert verify_data.artifacts.verification_result.num_scenes == 1
        assert verify_data.artifacts.verification_result.backend_run_count == 1
        assert "WORKER_BUSY" not in verify_resp.text

        busy_resp = await busy_task
        assert busy_resp.status_code == 200, busy_resp.text
        busy_data = BenchmarkToolResponse.model_validate(busy_resp.json())
        assert busy_data.success, busy_data.message


@pytest.mark.integration_p1
@pytest.mark.asyncio
@pytest.mark.int_id("INT-193B")
async def test_int_193b_worker_heavy_smoke_verification_uses_smoke_defaults(
    tmp_path, monkeypatch
):
    """INT-193B: smoke-mode verification should shrink omitted batch settings."""
    from shared.models.simulation import MultiRunResult
    from shared.workers.schema import ValidationResultRecord, VerificationRequest
    from worker_heavy.utils import verification as verification_mod

    root = tmp_path / "session"
    root.mkdir()
    (root / "scene.xml").write_text("<xml />\n", encoding="utf-8")
    (root / "validation_results.json").write_text(
        ValidationResultRecord(
            success=True,
            message="seeded",
            timestamp=0.0,
            script_path="script.py",
            script_sha256="0" * 64,
            verification_result=None,
        ).model_dump_json(),
        encoding="utf-8",
    )

    captured: dict[str, object] = {}

    class DummyBuilder:
        def build_from_assembly(self, component, objectives, smoke_test_mode):
            return root / "scene.xml"

    def fake_verify_with_jitter(**kwargs):
        captured.update(kwargs)
        return MultiRunResult(
            num_scenes=kwargs["num_scenes"],
            success_count=kwargs["num_scenes"],
            success_rate=1.0,
            is_consistent=True,
            individual_results=[],
            fail_reasons=[],
        )

    monkeypatch.setattr(
        verification_mod, "_load_workspace_benchmark_definition", lambda *_, **__: None
    )
    monkeypatch.setattr(
        verification_mod, "load_component_from_script", lambda *_, **__: object()
    )
    monkeypatch.setattr(
        verification_mod, "get_simulation_builder", lambda *_, **__: DummyBuilder()
    )
    monkeypatch.setattr(verification_mod, "verify_with_jitter", fake_verify_with_jitter)
    monkeypatch.setattr(
        verification_mod, "collect_and_cleanup_events", lambda *_, **__: []
    )
    monkeypatch.setattr(
        verification_mod, "record_validation_result", lambda *_, **__: None
    )

    response = await verification_mod.run_verification_job(
        root=root,
        request=VerificationRequest(
            script_path="script.py",
            backend=SimulatorBackendType.MUJOCO,
            smoke_test_mode=True,
            jitter_range=[0.0, 0.0, 0.0],
            seed=42,
        ),
        session_id="INT-193B",
    )

    assert response.success is True
    assert response.artifacts is not None
    assert response.artifacts.verification_result is not None
    assert response.artifacts.verification_result.num_scenes == 1
    assert response.artifacts.verification_result.backend_run_count == 1
    assert captured["num_scenes"] == 1
    assert captured["duration"] == 1.0
    assert captured["smoke_test_mode"] is True
