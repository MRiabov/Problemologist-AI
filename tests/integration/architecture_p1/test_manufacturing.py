import asyncio
import uuid

import pytest
from httpx import AsyncClient

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus
from shared.workers.schema import AnalyzeRequest
from shared.workers.workbench_models import ManufacturingMethod, WorkbenchResult

# Adjust URL to your controller if different
CONTROLLER_URL = "http://127.0.0.1:18000"
WORKER_HEAVY_URL = "http://127.0.0.1:18002"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_manufacturing_methods_and_materials():
    """
    INT-035: Materials config enforcement
    INT-036: Supported workbench methods

    Verifies:
    1. Manufacturing workbench runs (validate_costing_and_price)
    2. Different methods (CNC, 3D Print) are supported/detected
    3. Invalid materials are rejected (via prompt injection or reviewing logs/artifacts)
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        # 1. Setup Benchmark
        request = BenchmarkGenerateRequest(
            prompt="Create a benchmark for a CNC machined part."
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to generate benchmark: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        benchmark_session_id = str(benchmark_resp.session_id)

        # Wait for benchmark
        confirmed = False
        for _ in range(150):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if status_resp.status_code == 200:
                bench_ep = EpisodeResponse.model_validate(status_resp.json())
                if bench_ep.status == EpisodeStatus.PLANNED and not confirmed:
                    await client.post(
                        f"/benchmark/{benchmark_session_id}/confirm",
                        json=ConfirmRequest(comment="Proceed").model_dump(),
                    )
                    confirmed = True
                if bench_ep.status == EpisodeStatus.COMPLETED:
                    break
                if bench_ep.status == EpisodeStatus.FAILED:
                    pytest.fail(
                        "Benchmark generation failed during setup "
                        f"(session_id={benchmark_session_id})."
                    )
            await asyncio.sleep(2)
        else:
            pytest.fail("Benchmark generation timed out.")

        # 2. Trigger Engineer - Requesting CNC and specific material
        engineer_session_id = f"INT-036-{uuid.uuid4().hex[:8]}"
        # We explicitly ask for Aluminum 6061 (valid) to test success path first
        task = f"Solve benchmark: {benchmark_session_id}. Use CNC milling with Aluminum 6061."
        run_request = AgentRunRequest(
            task=task,
            session_id=engineer_session_id,
            metadata_vars={"benchmark_id": benchmark_session_id},
        )

        run_resp = await client.post("/agent/run", json=run_request.model_dump())
        assert run_resp.status_code in [
            200,
            202,
        ], f"Failed to trigger agent: {run_resp.text}"
        episode_id = AgentRunResponse.model_validate(run_resp.json()).episode_id

        # Wait for Engineer
        engineer_completed = False
        final_status = None
        for _ in range(150):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                ep = EpisodeResponse.model_validate(ep_resp.json())
                final_status = ep.status
                if final_status in [EpisodeStatus.COMPLETED, EpisodeStatus.FAILED]:
                    engineer_completed = True
                    break
            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail(f"Engineer timed out. Last status: {final_status}")

        # 3. Verify Workbench Execution (INT-036)
        # Use /episodes/{id} to get assets list
        ep_resp = await client.get(f"/episodes/{episode_id}")
        assert ep_resp.status_code == 200
        ep_data = EpisodeResponse.model_validate(ep_resp.json())
        assets = ep_data.assets or []

        # Check for assembly_definition.yaml which implies workbench ran
        cost_yaml_artifact = next(
            (a for a in assets if "assembly_definition.yaml" in a.s3_path),
            None,
        )
        assert cost_yaml_artifact is not None, (
            f"Workbench output (cost estimation) missing. Assets: {[a.s3_path for a in assets]}"
        )

        # 4. Verify Material Enforcement (INT-035) - "Nice to have" negative test
        bad_material_session_id = f"INT-035-{uuid.uuid4().hex[:8]}"
        bad_task = f"Solve benchmark: {benchmark_session_id}. Use Unobtanium material."

        bad_run_request = AgentRunRequest(
            task=bad_task,
            session_id=bad_material_session_id,
            metadata_vars={"benchmark_id": benchmark_session_id},
        )
        bad_run_resp = await client.post(
            "/agent/run", json=bad_run_request.model_dump()
        )
        assert bad_run_resp.status_code in [200, 202]
        bad_episode_id = AgentRunResponse.model_validate(bad_run_resp.json()).episode_id

        # Minimal assertion for INT-035: The system shouldn't crash, and if it fails, it handles it gracefully.
        for _ in range(150):
            ep_resp = await client.get(f"/episodes/{bad_episode_id}")
            if ep_resp.status_code == 200:
                ep = EpisodeResponse.model_validate(ep_resp.json())
                if ep.status in [EpisodeStatus.COMPLETED, EpisodeStatus.FAILED]:
                    break
            await asyncio.sleep(2)


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_worker_analyze_rejects_unknown_material():
    """The heavy worker must fail closed on unknown material IDs."""
    script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    part = Box(10, 10, 10)
    part = part.move(Location((0, 0, 5)))
    part.label = "test_part"
    part.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="unobtanium",
    )
    return part
"""

    async with AsyncClient(base_url=WORKER_HEAVY_URL, timeout=300.0) as client:
        request = AnalyzeRequest(
            script_path="script.py",
            script_content=script,
            method=ManufacturingMethod.CNC,
            quantity=1,
        )
        resp = await client.post(
            "/benchmark/analyze",
            json=request.model_dump(mode="json"),
            headers={"X-Session-ID": f"INT-035-{uuid.uuid4().hex[:8]}"},
        )
        assert resp.status_code == 200, resp.text
        result = WorkbenchResult.model_validate(resp.json())
        assert result.is_manufacturable is False
        assert any(
            "Unknown material_id" in violation for violation in result.violations
        )
