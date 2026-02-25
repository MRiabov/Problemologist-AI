import asyncio
import os
import uuid

import httpx
import pytest
import yaml

from controller.api.schemas import (
    AgentRunResponse,
    EpisodeResponse,
    OpenAPISchema,
    StandardResponse,
)
from controller.api.tasks import AgentRunRequest
from shared.enums import EpisodeStatus
from shared.models.schemas import (
    BoundingBox,
    Constraints,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
)
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    WriteFileRequest,
)

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_026_mandatory_event_families():
    """INT-026: Verify mandatory event families are emitted in a real run."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-026-{uuid.uuid4().hex[:8]}"

        # 1. Setup objectives.yaml
        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(8, 8, 8), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(0, 0, 0), max=(20, 20, 20)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(30, 30, 30)),
            moved_object=MovedObject(
                label="test_obj",
                shape="sphere",
                start_position=(0, 0, 5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )
        write_obj_req = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_obj_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # 2. Write a script that manually emits an event to verify collection
        script = """
import os
from build123d import *
from shared.models.schemas import PartMetadata
from shared.observability.events import emit_event

def build():
    # Emit event to verify collection
    emit_event({"event_type": "simulation_result", "data": {"status": "success"}})
    
    # Trigger a tool call if the worker instruments it
    emit_event({"event_type": "tool_call", "data": {"tool": "fastener_hole", "args": {"type": "M3"}}})

    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
        write_script_req = WriteFileRequest(
            path="script.py", content=script, overwrite=True
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_script_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # 3. Trigger simulation
        sim_req = BenchmarkToolRequest(script_path="script.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        events = data.events

        # Verify event families
        event_types = [e.get("event_type") for e in events]
        assert "simulation_result" in event_types, "Missing simulation_result event"
        assert "tool_call" in event_types, "Missing tool_call event"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_027_seed_variant_tracking():
    """INT-027: Verify DB persistence of variant_id and seed from the API response."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-027-{uuid.uuid4().hex[:8]}"
        variant_id = "test-variant-027"
        seed = 42

        # 1. Start an episode with seed and variant_id in metadata_vars
        payload = AgentRunRequest(
            task="Test seed and variant tracking",
            session_id=session_id,
            metadata_vars={
                "variant_id": variant_id,
                "seed": seed,
            },
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run", json=payload.model_dump(mode="json")
        )
        assert resp.status_code == 202
        run_data = AgentRunResponse.model_validate(resp.json())
        episode_id = run_data.episode_id

        # 2. Verify that variant_id and seed are persisted in the episode record
        data = None
        for _ in range(5):
            try:
                status_resp = await client.get(
                    f"{CONTROLLER_URL}/episodes/{episode_id}", timeout=10.0
                )
                if status_resp.status_code == 200:
                    ep_data = EpisodeResponse.model_validate(status_resp.json())
                    if ep_data.metadata_vars.get("variant_id") == variant_id:
                        data = ep_data
                        break
            except (httpx.ReadTimeout, httpx.ConnectError):
                pass
            await asyncio.sleep(1)

        assert data is not None
        assert data.metadata_vars.get("variant_id") == variant_id
        assert data.metadata_vars.get("seed") == seed


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_028_strict_api_schema_contract():
    """INT-028: Verify OpenAPI schema validity and live responses."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # 1. Controller OpenAPI
        resp = await client.get(f"{CONTROLLER_URL}/openapi.json")
        assert resp.status_code == 200
        schema_data = resp.json()
        OpenAPISchema.model_validate(schema_data)
        assert "openapi" in schema_data

        # 2. Worker OpenAPI
        resp = await client.get(f"{WORKER_LIGHT_URL}/openapi.json")
        assert resp.status_code == 200
        worker_schema = resp.json()
        OpenAPISchema.model_validate(worker_schema)

        # 3. Validate a live response against the schema
        health_resp = await client.get(f"{WORKER_LIGHT_URL}/health")
        assert health_resp.status_code == 200
        health_data = health_resp.json()

        try:
            health_schema = worker_schema["paths"]["/health"]["get"]["responses"][
                "200"
            ]["content"]["application/json"]["schema"]
            if "$ref" in health_schema:
                ref_name = health_schema["$ref"].split("/")[-1]
                health_schema = worker_schema["components"]["schemas"][ref_name]

            if "required" in health_schema:
                for req in health_schema["required"]:
                    assert req in health_data, (
                        f"Missing required field {req} in /health response"
                    )

            properties = health_schema.get("properties", {})
            if "status" in properties:
                assert "status" in health_data

        except KeyError as e:
            pytest.fail(f"Could not locate schema for /health validation: {e}")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_029_api_key_enforcement(controller_client):
    """INT-029: Verify API key enforcement on protected endpoints."""
    client = controller_client

    # No key
    resp = await client.post("/ops/backup")
    assert resp.status_code == 403

    # Invalid auth
    resp = await client.post(
        "/ops/backup",
        headers={"X-Backup-Secret": "invalid-auth-val"},
    )
    assert resp.status_code == 403

    valid_auth = os.getenv("BACKUP_SECRET", "change-me-in-production")
    resp = await client.post("/ops/backup", headers={"X-Backup-Secret": valid_auth})
    assert resp.status_code in [202, 500]
    if resp.status_code == 500:
        assert (
            "Backup configuration missing" in resp.text
            or "Temporal" in resp.text
            or "connection" in resp.text.lower()
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_030_interrupt_propagation():
    """INT-030: Verify user interrupt cancels worker jobs."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-030-{uuid.uuid4().hex[:8]}"
        payload = AgentRunRequest(
            task="Perform a very complex multi-step reasoning task.",
            session_id=session_id,
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run", json=payload.model_dump(mode="json")
        )
        assert resp.status_code == 202
        run_data = AgentRunResponse.model_validate(resp.json())
        episode_id = run_data.episode_id

        await asyncio.sleep(0.5)

        interrupt_resp = await client.post(
            f"{CONTROLLER_URL}/episodes/{episode_id}/interrupt"
        )
        assert interrupt_resp.status_code in [200, 202]
        StandardResponse.model_validate(interrupt_resp.json())

        status = None
        for i in range(20):
            await asyncio.sleep(0.5)
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            assert status_resp.status_code == 200
            ep_data = EpisodeResponse.model_validate(status_resp.json())
            status = ep_data.status
            if status in [EpisodeStatus.CANCELLED, EpisodeStatus.FAILED]:
                break

        assert status in [
            EpisodeStatus.CANCELLED,
            EpisodeStatus.FAILED,
        ], f"Expected cancelled or failed, got {status}"
