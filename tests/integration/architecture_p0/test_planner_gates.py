import asyncio
import uuid
import pytest
from httpx import AsyncClient

# Worker URL (assuming it's running in compose)
WORKER_URL = "http://localhost:8001"


@pytest.fixture
async def async_client():
    async with AsyncClient(base_url=WORKER_URL, timeout=10.0) as client:
        yield client


@pytest.fixture
def session_id():
    return str(uuid.uuid4())


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


async def setup_workspace(client, headers, files):
    """Utility to setup a workspace with specific files."""
    for path, content in files.items():
        resp = await client.post(
            "/fs/write", json={"path": path, "content": content}, headers=headers
        )
        assert resp.status_code == 200


def get_valid_plan():
    return """# Engineering Plan
## 1. Solution Overview
Simple test plan.
## 2. Parts List
- Part A
## 3. Assembly Strategy
1. Step 1
## 4. Cost & Weight Budget
- Item 1: $10
## 5. Risk Assessment
- Risk 1
"""


def get_valid_todo():
    return "- [ ] Task 1"


def get_valid_objectives():
    return """
objectives:
  goal_zone:
    min: [10, 10, 10]
    max: [20, 20, 20]
  forbid_zones: []
  build_zone:
    min: [0, 0, 0]
    max: [100, 100, 100]

simulation_bounds:
  min: [-10, -10, -10]
  max: [110, 110, 110]

moved_object:
  label: "ball"
  shape: "sphere"
  static_randomization:
    radius: [5, 5]
  start_position: [50, 50, 50]
  runtime_jitter: [0, 0, 0]

moving_parts: []

constraints:
  max_unit_cost: 100.0
  max_weight: 10.0

randomization:
  static_variation_id: "test_v1"
  runtime_jitter_enabled: false
"""


def get_valid_cost():
    return """
final_assembly:
  - part_1:
      method: "cnc"
      material: "aluminum-6061"
      quantity: 1
"""


def get_basic_script():
    return "def build(): return None"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_005_mandatory_artifacts(async_client, base_headers):
    """INT-005: Engineer planner mandatory artifact gate."""
    # Start with empty workspace
    # 1. Missing plan.md
    files = {
        "todo.md": get_valid_todo(),
        "objectives.yaml": get_valid_objectives(),
        "preliminary_cost_estimation.yaml": get_valid_cost(),
        "script.py": get_basic_script(),
    }
    await setup_workspace(async_client, base_headers, files)

    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    assert (
        resp.status_code == 200
    )  # Worker handles errors by returning success=False in body
    data = resp.json()
    assert data["success"] is False
    assert "plan.md is missing" in data["message"]

    # 2. Missing todo.md
    files["plan.md"] = get_valid_plan()
    del files["todo.md"]
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "todo.md is missing" in data["message"]

    # 3. Missing objectives.yaml
    files["todo.md"] = get_valid_todo()
    del files["objectives.yaml"]
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "objectives.yaml is missing" in data["message"]

    # 4. Missing preliminary_cost_estimation.yaml
    files["objectives.yaml"] = get_valid_objectives()
    del files["preliminary_cost_estimation.yaml"]
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "preliminary_cost_estimation.yaml is missing" in data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_006_plan_structure(async_client, base_headers):
    """INT-006: plan.md structure validation."""
    files = {
        "todo.md": get_valid_todo(),
        "objectives.yaml": get_valid_objectives(),
        "preliminary_cost_estimation.yaml": get_valid_cost(),
        "script.py": get_basic_script(),
    }

    # 1. Missing section
    invalid_plan = "# Engineering Plan\n## 1. Solution Overview\nOnly one section."
    files["plan.md"] = invalid_plan
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "Missing required section" in data["message"]

    # 2. Parts List missing bullet/table
    invalid_parts = get_valid_plan().replace("- Part A", "Just some text here.")
    files["plan.md"] = invalid_parts
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "Parts List must contain a bullet list or table" in data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_007_todo_integrity(async_client, base_headers):
    """INT-007: todo.md checkbox integrity."""
    files = {
        "plan.md": get_valid_plan(),
        "objectives.yaml": get_valid_objectives(),
        "preliminary_cost_estimation.yaml": get_valid_cost(),
        "script.py": get_basic_script(),
    }

    # 1. Invalid checkbox format
    files["todo.md"] = "- [?] Invalid"
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "invalid checkbox" in data["message"].lower()

    # 2. Uncompleted items
    files["todo.md"] = "- [ ] Uncompleted"
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "must be completed or skipped" in data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_008_objectives_validation(async_client, base_headers):
    """INT-008: objectives.yaml logic validation."""
    files = {
        "plan.md": get_valid_plan(),
        "todo.md": "- [x] Done",
        "preliminary_cost_estimation.yaml": get_valid_cost(),
        "script.py": get_basic_script(),
    }

    # 1. Schema violation (missing required field)
    files["objectives.yaml"] = "objectives: {}"
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "objectives.yaml invalid" in data["message"]

    # 2. Template placeholders
    placeholder_obj = get_valid_objectives().replace(
        "100, 100, 100", "x_min, y_min, z_min"
    )
    files["objectives.yaml"] = placeholder_obj
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "template placeholders" in data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_009_cost_estimation_validation(async_client, base_headers):
    """INT-009: preliminary_cost_estimation.yaml schema gate."""
    files = {
        "plan.md": get_valid_plan(),
        "todo.md": "- [x] Done",
        "objectives.yaml": get_valid_objectives(),
        "script.py": get_basic_script(),
    }

    # 1. Schema violation
    files["preliminary_cost_estimation.yaml"] = "invalid_yaml: true"
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "preliminary_cost_estimation.yaml invalid" in data["message"]

    # 2. Template placeholders
    placeholder_cost = get_valid_cost().replace("part_1", "ramp_main")
    files["preliminary_cost_estimation.yaml"] = placeholder_cost
    await setup_workspace(async_client, base_headers, files)
    resp = await async_client.post(
        "/benchmark/submit", json={"script_path": "script.py"}, headers=base_headers
    )
    data = resp.json()
    assert data["success"] is False
    assert "template placeholders" in data["message"]
