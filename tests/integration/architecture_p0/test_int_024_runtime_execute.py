import os
import uuid

import httpx
import pytest
import yaml

from shared.models.schemas import (
    BoundingBox,
    Constraints,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
)
from shared.workers.schema import ExecuteRequest, ExecuteResponse, WriteFileRequest

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")


def _runtime_validate_code() -> str:
    return (
        "import importlib.util\n"
        "from shared.utils.agent import validate\n"
        'spec = importlib.util.spec_from_file_location("benchmark_script", "script.py")\n'
        "mod = importlib.util.module_from_spec(spec)\n"
        "spec.loader.exec_module(mod)\n"
        "compound = mod.build()\n"
        "success, message = validate(compound)\n"
        'print(f"VALIDATE_SUCCESS={success}")\n'
        'print(f"VALIDATE_MESSAGE={message}")\n'
    )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_runtime_execute_reaches_benchmark_validate_toolchain():
    """
    INT-024: benchmark validation remains reachable from the light-worker
    runtime via shared.utils.agent.validate() inside execute_command-equivalent
    execution, without host-resolution failures.
    """
    session_id = f"INT-024-RT-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata

def build():
    p = Box(4, 4, 4).move(Location((0, 0, 4)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""

    invalid_objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(0.0, 0.0, 0.0), max=(10.0, 10.0, 10.0)),
            forbid_zones=[
                {
                    "name": "goal_overlap",
                    "min": (5.0, 5.0, 5.0),
                    "max": (12.0, 12.0, 12.0),
                }
            ],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="sphere",
            start_position=(0.0, 0.0, 4.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
    )

    valid_objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(8.0, 8.0, 0.0), max=(12.0, 12.0, 8.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="sphere",
            start_position=(0.0, 0.0, 4.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
    )

    async with httpx.AsyncClient(timeout=300.0) as client:
        write_script = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py",
                content=script,
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_script.status_code == 200, write_script.text

        write_invalid = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=yaml.dump(invalid_objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_invalid.status_code == 200, write_invalid.text

        invalid_exec = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=_runtime_validate_code(),
                timeout=120,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert invalid_exec.status_code == 200, invalid_exec.text
        invalid_data = ExecuteResponse.model_validate(invalid_exec.json())
        assert invalid_data.exit_code == 0
        assert "VALIDATE_SUCCESS=False" in invalid_data.stdout
        assert "goal_zone overlaps forbid zone" in invalid_data.stdout
        assert "Temporary failure in name resolution" not in invalid_data.stdout

        write_valid = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=yaml.dump(valid_objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_valid.status_code == 200, write_valid.text

        valid_exec = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=_runtime_validate_code(),
                timeout=120,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert valid_exec.status_code == 200, valid_exec.text
        valid_data = ExecuteResponse.model_validate(valid_exec.json())
        assert valid_data.exit_code == 0
        assert "VALIDATE_SUCCESS=True" in valid_data.stdout
        assert "Temporary failure in name resolution" not in valid_data.stdout


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_runtime_validate_rejects_parent_only_fixed_metadata():
    """
    INT-024: benchmark validation fails closed when a parent Compound is marked
    fixed but the static child fixtures themselves are left unfixed.
    """
    session_id = f"INT-024-FIX-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    script = """
from build123d import Align, Box, BuildPart, Compound
from shared.models.schemas import CompoundMetadata, PartMetadata

def build():
    with BuildPart() as ground_builder:
        Box(4, 4, 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ground = ground_builder.part
    ground.label = "ground_plane"
    ground.metadata = PartMetadata(material_id="hdpe")

    with BuildPart() as ball_builder:
        Box(0.4, 0.4, 0.4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ball = ball_builder.part
    ball.label = "target_box"
    ball.metadata = PartMetadata(material_id="steel_bearing", fixed=False)

    benchmark = Compound(children=[ground, ball])
    benchmark.label = "benchmark_assembly"
    benchmark.metadata = CompoundMetadata(fixed=True)
    return benchmark
"""

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(8.0, 8.0, 0.0), max=(12.0, 12.0, 8.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="box",
            start_position=(0.0, 0.0, 0.2),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
    )

    async with httpx.AsyncClient(timeout=300.0) as client:
        write_script = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py",
                content=script,
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_script.status_code == 200, write_script.text

        write_objectives = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=yaml.dump(objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_objectives.status_code == 200, write_objectives.text

        exec_response = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=_runtime_validate_code(),
                timeout=120,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert exec_response.status_code == 200, exec_response.text
        data = ExecuteResponse.model_validate(exec_response.json())
        assert data.exit_code == 0
        assert "VALIDATE_SUCCESS=False" in data.stdout
        assert "does not make child parts static" in data.stdout


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_runtime_validate_rejects_translated_top_level_parts():
    """
    INT-024: benchmark validation fails closed when top-level parts are placed
    with translated geometry instead of location-based transforms.
    """
    session_id = f"INT-024-LOC-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    script = """
from build123d import Align, Box, BuildPart, Compound
from shared.models.schemas import CompoundMetadata, PartMetadata

def build():
    with BuildPart() as ground_builder:
        Box(4, 4, 0.2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ground = ground_builder.part
    ground.label = "ground_plane"
    ground.metadata = PartMetadata(material_id="hdpe", fixed=True)

    with BuildPart() as platform_builder:
        Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    platform = platform_builder.part.translate((8, 0, 0))
    platform.label = "platform_left"
    platform.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    with BuildPart() as ball_builder:
        Box(0.4, 0.4, 0.4, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ball = ball_builder.part.translate((8, 0, 1.2))
    ball.label = "target_box"
    ball.metadata = PartMetadata(material_id="steel_bearing", fixed=False)

    benchmark = Compound(children=[ground, platform, ball])
    benchmark.label = "benchmark_assembly"
    benchmark.metadata = CompoundMetadata()
    return benchmark
"""

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10.0, -2.0, 0.0), max=(12.0, 2.0, 4.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
        ),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="target_box",
            shape="box",
            start_position=(8.0, 0.0, 1.2),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
    )

    async with httpx.AsyncClient(timeout=300.0) as client:
        write_script = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py",
                content=script,
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_script.status_code == 200, write_script.text

        write_objectives = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=yaml.dump(objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_objectives.status_code == 200, write_objectives.text

        exec_response = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=_runtime_validate_code(),
                timeout=120,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert exec_response.status_code == 200, exec_response.text
        data = ExecuteResponse.model_validate(exec_response.json())
        assert data.exit_code == 0
        assert "VALIDATE_SUCCESS=False" in data.stdout
        assert "Use `.move(Location(...))` or `.moved(Location(...))`" in data.stdout


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_runtime_validate_reports_resolved_build_zone_bounds():
    """
    INT-024: build-zone violations must report the resolved objective bounds,
    not `build_zone None`, so the agent gets actionable correction feedback.
    """
    session_id = f"INT-024-BZ-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    script = """
from build123d import Align, Box
from shared.models.schemas import PartMetadata

def build():
    ground = Box(40, 40, 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ground.label = "ground_plane"
    ground.metadata = PartMetadata(material_id="hdpe", fixed=True)
    return ground
"""

    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(8.0, 8.0, 0.0), max=(12.0, 12.0, 8.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-5.0, -5.0, 0.0), max=(5.0, 5.0, 10.0)),
        ),
        simulation_bounds=BoundingBox(
            min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
        ),
        moved_object=MovedObject(
            label="ground_plane",
            shape="box",
            start_position=(0.0, 0.0, 0.5),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
    )

    async with httpx.AsyncClient(timeout=300.0) as client:
        write_script = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py",
                content=script,
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_script.status_code == 200, write_script.text

        write_objectives = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=yaml.dump(objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_objectives.status_code == 200, write_objectives.text

        exec_response = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=_runtime_validate_code(),
                timeout=120,
            ).model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert exec_response.status_code == 200, exec_response.text
        data = ExecuteResponse.model_validate(exec_response.json())
        assert data.exit_code == 0
        assert "VALIDATE_SUCCESS=False" in data.stdout
        assert "build_zone None" not in data.stdout
        assert "'min': (-5.0, -5.0, 0.0)" in data.stdout
