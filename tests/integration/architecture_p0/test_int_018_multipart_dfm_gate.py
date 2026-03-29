import asyncio
import os
import uuid

import httpx
import pytest

from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    WriteFileRequest,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


async def _wait_for_heavy_ready(client: httpx.AsyncClient) -> None:
    for _ in range(120):
        ready_resp = await client.get(f"{WORKER_HEAVY_URL}/ready", timeout=5.0)
        if ready_resp.status_code == 200:
            return
        await asyncio.sleep(0.5)
    pytest.fail("worker-heavy did not become ready before INT-018 request")


async def _post_heavy_tool(
    client: httpx.AsyncClient,
    path: str,
    request: BenchmarkToolRequest,
    headers: dict[str, str],
) -> httpx.Response:
    for _ in range(120):
        await _wait_for_heavy_ready(client)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}{path}",
            json=request.model_dump(mode="json"),
            headers=headers,
        )
        if resp.status_code != 503:
            return resp
        await asyncio.sleep(0.5)
    pytest.fail(f"worker-heavy stayed busy for {path}")


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(regexes=["simulation_failed"])
@pytest.mark.asyncio
@pytest.mark.int_id("INT-018")
async def test_int_018_simulate_does_not_dfm_reject_simple_multipart_benchmark():
    """
    INT-018: benchmark-owned multipart fixtures must not be DFM-gated during
    simulate when no engineer-owned manufactured-part handoff exists.
    """
    session_id = f"INT-018-MP-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    script = """
from build123d import Align, Box, BuildPart, Compound, Location, Sphere
from shared.models.schemas import CompoundMetadata, PartMetadata

def build():
    with BuildPart() as ground_builder:
        Box(0.6, 0.18, 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ground = ground_builder.part
    ground.label = "ground_plane"
    ground.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as left_wall_builder:
        Box(0.08, 0.02, 0.06, align=(Align.CENTER, Align.CENTER, Align.MIN))
    left_wall = left_wall_builder.part.move(Location((-0.18, -0.05, 0.0)))
    left_wall.label = "guide_left"
    left_wall.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as right_wall_builder:
        Box(0.08, 0.02, 0.06, align=(Align.CENTER, Align.CENTER, Align.MIN))
    right_wall = right_wall_builder.part.move(Location((0.18, 0.05, 0.0)))
    right_wall.label = "guide_right"
    right_wall.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    ball = Sphere(0.02).move(Location((0.0, 0.0, 0.03)))
    ball.label = "projectile_ball"
    ball.metadata = PartMetadata(material_id="steel_bearing", fixed=False)

    assembly = Compound(children=[ground, left_wall, right_wall, ball])
    assembly.label = "multipart_probe"
    assembly.metadata = CompoundMetadata()
    return assembly
"""

    objectives = """
objectives:
  goal_zone:
    min: [-0.03, -0.03, 0.0]
    max: [0.03, 0.03, 0.06]
  forbid_zones: []
  build_zone:
    min: [-0.35, -0.1, 0.0]
    max: [0.35, 0.1, 0.1]
  fluid_objectives: []
  stress_objectives: []
physics:
  backend: GENESIS
  fem_enabled: false
fluids: []
simulation_bounds:
  min: [-0.4, -0.12, 0.0]
  max: [0.4, 0.12, 0.12]
moved_object:
  label: projectile_ball
  shape: sphere
  material_id: abs
  static_randomization:
    radius: [0.02, 0.02]
  start_position: [0.0, 0.0, 0.03]
  runtime_jitter: [0.0, 0.0, 0.0]
constraints:
  max_unit_cost: 120.0
  max_weight_g: 5000.0
benchmark_parts:
  - part_id: ground_plane
    label: ground_plane
    metadata:
      fixed: true
      material_id: hdpe
randomization:
  static_variation_id: v1.0
  runtime_jitter_enabled: false
"""

    async with httpx.AsyncClient(timeout=300.0) as client:
        for path, content in {
            "script.py": script,
            "benchmark_definition.yaml": objectives,
        }.items():
            resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path=path,
                    content=content,
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert resp.status_code == 200, resp.text

        validate_resp = await _post_heavy_tool(
            client,
            "/benchmark/validate",
            BenchmarkToolRequest(script_path="script.py"),
            headers,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success is True, validate_data.message

        simulate_resp = await _post_heavy_tool(
            client,
            "/benchmark/simulate",
            BenchmarkToolRequest(script_path="script.py"),
            headers,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success is True, simulate_data.message
        assert simulate_data.message is not None
        assert "undercut faces detected" not in simulate_data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(regexes=["simulation_failed"])
@pytest.mark.asyncio
@pytest.mark.int_id("INT-018")
async def test_int_018_simulation_bounds_ignore_fixed_benchmark_fixtures():
    """
    INT-018: simulation_bounds apply to moving bodies, not fixed benchmark
    fixtures. A fixed obstacle outside the moved-object envelope must not
    trigger OUT_OF_BOUNDS on its own.
    """
    session_id = f"INT-018-BOUNDS-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    script = """
from build123d import Align, Box, BuildPart, Compound, Location, Sphere
from shared.models.schemas import CompoundMetadata, PartMetadata

def build():
    with BuildPart() as ground_builder:
        Box(0.8, 0.18, 0.02, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ground = ground_builder.part
    ground.label = "ground_plane"
    ground.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as blocker_builder:
        Box(0.04, 0.04, 0.10, align=(Align.CENTER, Align.CENTER, Align.MIN))
    blocker = blocker_builder.part.move(Location((0.5, 0.0, 0.02)))
    blocker.label = "fixed_blocker"
    blocker.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    ball = Sphere(0.02).move(Location((0.0, 0.0, 0.03)))
    ball.label = "projectile_ball"
    ball.metadata = PartMetadata(material_id="steel_bearing", fixed=False)

    assembly = Compound(children=[ground, blocker, ball])
    assembly.label = "bounds_probe"
    assembly.metadata = CompoundMetadata()
    return assembly
"""

    objectives = """
objectives:
  goal_zone:
    min: [-0.03, -0.03, 0.0]
    max: [0.03, 0.03, 0.06]
  forbid_zones: []
  build_zone:
    min: [-0.6, -0.1, 0.0]
    max: [0.6, 0.1, 0.14]
  fluid_objectives: []
  stress_objectives: []
physics:
  backend: GENESIS
  fem_enabled: false
fluids: []
simulation_bounds:
  min: [-0.2, -0.12, 0.0]
  max: [0.2, 0.12, 0.12]
moved_object:
  label: projectile_ball
  shape: sphere
  material_id: abs
  static_randomization:
    radius: [0.02, 0.02]
  start_position: [0.0, 0.0, 0.03]
  runtime_jitter: [0.0, 0.0, 0.0]
constraints:
  max_unit_cost: 120.0
  max_weight_g: 5000.0
benchmark_parts:
  - part_id: ground_plane
    label: ground_plane
    metadata:
      fixed: true
      material_id: hdpe
randomization:
  static_variation_id: v1.0
  runtime_jitter_enabled: false
"""

    async with httpx.AsyncClient(timeout=300.0) as client:
        for path, content in {
            "script.py": script,
            "benchmark_definition.yaml": objectives,
        }.items():
            resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(
                    path=path,
                    content=content,
                    overwrite=True,
                ).model_dump(mode="json"),
                headers=headers,
            )
            assert resp.status_code == 200, resp.text

        validate_resp = await _post_heavy_tool(
            client,
            "/benchmark/validate",
            BenchmarkToolRequest(script_path="script.py"),
            headers,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success is True, validate_data.message

        simulate_resp = await _post_heavy_tool(
            client,
            "/benchmark/simulate",
            BenchmarkToolRequest(script_path="script.py"),
            headers,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success is True, simulate_data.message
        assert simulate_data.message is not None
        assert "OUT_OF_BOUNDS:fixed_blocker" not in simulate_data.message
