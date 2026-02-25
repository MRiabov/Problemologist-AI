import os
import uuid

import httpx
import pytest
import yaml

from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    BoundingBox,
    Constraints,
    CostTotals,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
)
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    DeleteFileRequest,
    WriteFileRequest,
)

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.fixture
def session_id():
    return f"test-gates-{uuid.uuid4().hex[:8]}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


@pytest.fixture
def valid_plan():
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


@pytest.fixture
def valid_todo():
    return "- [x] Step 1\n- [-] Step 2"


@pytest.fixture
def valid_objectives():
    return ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(10.0, 10.0, 10.0), max=(20.0, 20.0, 20.0)),
            forbid_zones=[],
            build_zone=BoundingBox(min=(-50.0, -50.0, 0.0), max=(50.0, 50.0, 100.0)),
        ),
        simulation_bounds=BoundingBox(
            min=(-100.0, -100.0, 0.0), max=(100.0, 100.0, 100.0)
        ),
        moved_object=MovedObject(
            label="ball",
            shape="sphere",
            start_position=(0.0, 0.0, 50.0),
            runtime_jitter=(0.0, 0.0, 0.0),
        ),
        constraints=Constraints(max_unit_cost=50.0, max_weight_g=1.0),
    )


@pytest.fixture
def valid_cost():
    return AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=50.0,
            benchmark_max_weight_g=1000.0,
            planner_target_max_unit_cost_usd=45.0,
            planner_target_max_weight_g=900.0,
        ),
        totals=CostTotals(
            estimated_unit_cost_usd=30.0,
            estimated_weight_g=500.0,
            estimate_confidence="high",
        ),
    )


@pytest.fixture
def minimal_script():
    return """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    # Box 10x10x10 centered at (0,0,5) -> Z from 0 to 10.
    # Build zone is [0, 100] in objectives.
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""


async def setup_workspace(client, headers, files):
    """Utility to setup a workspace. Overwrites if exists."""
    for path, content in files.items():
        if isinstance(content, (ObjectivesYaml, AssemblyDefinition)):
            content_str = yaml.dump(content.model_dump(mode="json", by_alias=True))
        elif not isinstance(content, str):
            content_str = yaml.dump(content)
        else:
            content_str = content

        write_req = WriteFileRequest(
            path=path,
            content=content_str,
            overwrite=True,
        )
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_req.model_dump(mode="json"),
            headers=headers,
        )
        assert resp.status_code == 200, f"Failed to write {path}: {resp.text}"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_005_mandatory_artifacts_gate(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
    minimal_script,
):
    """INT-005: Verify rejection if mandatory artifacts are missing."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Initial: All required files except one
        base_files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Missing plan.md
        files = base_files.copy()
        del files["plan.md"]
        await setup_workspace(client, base_headers, files)
        delete_req = DeleteFileRequest(path="plan.md")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_req.model_dump(mode="json"),
            headers=base_headers,
        )

        submit_req = BenchmarkToolRequest(script_path="solution.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "plan.md is missing" in data.message

        # 2. Missing todo.md
        files = base_files.copy()
        del files["todo.md"]
        await setup_workspace(client, base_headers, files)
        delete_todo_req = DeleteFileRequest(path="todo.md")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_todo_req.model_dump(mode="json"),
            headers=base_headers,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "todo.md is missing" in data.message

        # 3. Missing assembly_definition.yaml
        files = base_files.copy()
        del files["assembly_definition.yaml"]
        await setup_workspace(client, base_headers, files)
        delete_cost_req = DeleteFileRequest(path="assembly_definition.yaml")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_cost_req.model_dump(mode="json"),
            headers=base_headers,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "assembly_definition.yaml is missing" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_006_plan_structure_validation(
    session_id, base_headers, valid_todo, valid_objectives, valid_cost, minimal_script
):
    """INT-006: Verify plan.md structural requirements."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Missing required heading
        invalid_plan = "## 1. Solution Overview\nNo other headings."
        await setup_workspace(
            client, base_headers, {**base_files, "plan.md": invalid_plan}
        )
        submit_req = BenchmarkToolRequest(script_path="solution.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "plan.md invalid" in data.message
        assert "Missing required section" in data.message

        # 2. Parts List missing table/bullets
        invalid_plan = """## 1. Solution Overview
Overview.
## 2. Parts List
Just some text here, no list or table.
## 3. Assembly Strategy
1. Step
## 4. Cost & Weight Budget
- Cost: 0
## 5. Risk Assessment
- Risk: None
"""
        await setup_workspace(
            client, base_headers, {**base_files, "plan.md": invalid_plan}
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "Parts List must contain a bullet list or table" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_007_todo_integrity(
    session_id, base_headers, valid_plan, valid_objectives, valid_cost, minimal_script
):
    """INT-007: Verify todo.md integrity (all items completed or skipped)."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "plan.md": valid_plan,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Invalid checkbox format
        invalid_todo = "- [?] What is this?"
        await setup_workspace(
            client, base_headers, {**base_files, "todo.md": invalid_todo}
        )
        submit_req = BenchmarkToolRequest(script_path="solution.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "todo.md invalid" in data.message
        assert "invalid checkbox" in data.message

        # 2. Uncompleted item at submission
        uncompleted_todo = "- [ ] I forgot to finish this"
        await setup_workspace(
            client, base_headers, {**base_files, "todo.md": uncompleted_todo}
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "must be completed or skipped" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_008_objectives_validation(
    session_id, base_headers, valid_plan, valid_todo, valid_cost, minimal_script
):
    """INT-008: Verify objectives.yaml schema and template detection."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }

        # 1. Template placeholders present (e.g., x_min)
        template_content = "objectives:\n  goal_zone:\n    min: [x_min, y_min, z_min]"
        await setup_workspace(
            client, base_headers, {**base_files, "objectives.yaml": template_content}
        )
        submit_req = BenchmarkToolRequest(script_path="solution.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "template placeholders" in data.message

        # 2. Schema violation (wrong type)
        invalid_obj = {"objectives": {"goal_zone": {"min": "not_a_list"}}}
        await setup_workspace(
            client, base_headers, {**base_files, "objectives.yaml": invalid_obj}
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "objectives.yaml invalid" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_009_cost_estimation_validation(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, minimal_script
):
    """INT-009: Verify assembly_definition.yaml schema and placeholders."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        base_files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "solution.py": minimal_script,
        }

        # 1. Template placeholders ([implement here])
        template_cost = "version: '1.0'\ntotals:\n  estimated_unit_cost_usd: 10.0\n  note: [implement here]"
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "assembly_definition.yaml": template_cost},
        )
        submit_req = BenchmarkToolRequest(script_path="solution.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "template placeholders" in data.message

        # 2. Schema violation (missing required field)
        invalid_cost = {
            "version": "1.0",
            "totals": {},
        }  # Missing estimated_unit_cost_usd
        await setup_workspace(
            client,
            base_headers,
            {**base_files, "assembly_definition.yaml": invalid_cost},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "assembly_definition.yaml invalid" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_011_planner_caps_enforcement(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, minimal_script
):
    """INT-011: Verify handoff blockage when planner caps exceed benchmark limits."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Create invalid cost estimation (target > benchmark cap)
        invalid_cost = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=50.0,
                benchmark_max_weight_g=1000.0,
                planner_target_max_unit_cost_usd=60.0,  # OVER LIMIT
                planner_target_max_weight_g=500.0,
            ),
            totals=CostTotals(
                estimated_unit_cost_usd=40.0,
                estimated_weight_g=200.0,
                estimate_confidence="medium",
            ),
        )
        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": invalid_cost,
            "solution.py": minimal_script,
        }
        await setup_workspace(client, base_headers, files)
        # Record validation
        val_req = BenchmarkToolRequest(script_path="solution.py")
        await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "assembly_definition.yaml invalid" in data.message
        assert (
            "Planner target cost (60.0) must be less than or equal to benchmark max cost (50.0)"
            in data.message
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_015_engineer_handover_immutability(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
    minimal_script,
):
    """INT-015: Verify immutability of objectives.yaml during handover."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await client.post(f"{WORKER_LIGHT_URL}/git/init", headers=base_headers)

        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }
        await setup_workspace(client, base_headers, files)

        # Baseline commit
        from shared.workers.schema import GitCommitRequest

        commit_req = GitCommitRequest(message="Initial benchmark")
        await client.post(
            f"{WORKER_LIGHT_URL}/git/commit",
            json=commit_req.model_dump(mode="json"),
            headers=base_headers,
        )

        # Cheat: modify objectives.yaml
        modified_objectives = valid_objectives.model_copy(deep=True)
        modified_objectives.constraints.max_unit_cost = 1000.0
        await setup_workspace(
            client, base_headers, {"objectives.yaml": modified_objectives}
        )
        # Record validation
        val_req = BenchmarkToolRequest(script_path="solution.py")
        await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "objectives.yaml violation" in data.message
        assert "has been modified" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_019_hard_constraints_gates(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, valid_cost
):
    """INT-019: Verify cost/weight/build-zone hard failure during submission."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Box translated to (1000, 1000, 1000) will be outside [ -50, 50 ]
        oob_script = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(10, 10, 10)
    p = p.move(Location((1000, 1000, 1000)))
    p.label = "oob_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum-6061"
    )
    return p
"""
        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": valid_cost,
            "solution.py": oob_script,
        }
        await setup_workspace(client, base_headers, files)
        # Record validation
        val_req = BenchmarkToolRequest(script_path="solution.py")
        await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "Submission rejected (DFM)" in data.message
        assert (
            "Build zone violation" in data.message
            or "out of bounds" in data.message.lower()
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_010_planner_pricing_script_integration(
    session_id, base_headers, valid_plan, valid_todo, valid_objectives, minimal_script
):
    """INT-010: Verify validate_costing_and_price block when over caps."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Create cost estimation where totals > planner caps (which are valid vs benchmark)
        invalid_cost = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=50.0,
                benchmark_max_weight_g=1000.0,
                planner_target_max_unit_cost_usd=45.0,
                planner_target_max_weight_g=900.0,
            ),
            totals=CostTotals(
                estimated_unit_cost_usd=55.0,  # OVER PLANNER CAP
                estimated_weight_g=500.0,
                estimate_confidence="high",
            ),
        )
        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": invalid_cost,
            "solution.py": minimal_script,
        }
        await setup_workspace(client, base_headers, files)
        # Record validation
        val_req = BenchmarkToolRequest(script_path="solution.py")
        await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "assembly_definition.yaml invalid" in data.message
        assert "exceeds target" in data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_018_validate_and_price_integration_gate(
    session_id,
    base_headers,
    valid_plan,
    valid_todo,
    valid_objectives,
    valid_cost,
    minimal_script,
):
    """INT-018: Verify simulate/submit require prior validation."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # 1. Setup workspace BUT do not call /benchmark/validate
        files = {
            "plan.md": valid_plan,
            "todo.md": valid_todo,
            "objectives.yaml": valid_objectives,
            "assembly_definition.yaml": valid_cost,
            "solution.py": minimal_script,
        }
        await setup_workspace(client, base_headers, files)

        # Let's delete the cached validation results specifically if any
        delete_req = DeleteFileRequest(path="validation_results.json")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/delete",
            json=delete_req.model_dump(mode="json"),
            headers=base_headers,
        )

        submit_req = BenchmarkToolRequest(script_path="solution.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=base_headers,
        )
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "validation" in data.message.lower()
