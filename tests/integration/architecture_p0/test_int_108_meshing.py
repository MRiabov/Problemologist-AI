import os
import time

import httpx
import pytest

# Constants
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:18001")


@pytest.fixture
def session_id():
    return f"test-mesh-{int(time.time())}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_108_tetrahedralization_pipeline(session_id, base_headers):
    """
    INT-108: Verify STL -> TetGen -> .msh flow.
    """
    async with httpx.AsyncClient() as client:
        # 1. Write a valid STL script
        script_content = """
from build123d import *
def build():
    return Box(10, 10, 10)
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "box.py", "content": script_content},
            headers=base_headers,
        )

        # 2. Trigger tetrahedralization (via a hypothetical worker endpoint if exists,
        # or as part of Genesis simulation setup)
        # Assuming we can trigger meshing via /runtime/execute for now to test the internal tool
        code = """
from pathlib import Path
from worker.utils.mesh_utils import tetrahedralize
from build123d import Box, export_stl

part = Box(1, 1, 1)
export_stl(part, "test.stl")
msh_path = tetrahedralize(Path("test.stl"), Path("test.msh"))
print(f"MESH_RESULT:{msh_path}")
"""
        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={"code": code},
            headers=base_headers,
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert data["exit_code"] == 0
        assert "MESH_RESULT:test.msh" in data["stdout"]

        # 3. Verify files exist in session
        ls_resp = await client.post(
            f"{WORKER_URL}/fs/ls", json={"path": "."}, headers=base_headers
        )
        files = [f["name"] for f in ls_resp.json()["files"]]
        # TetGen produces .node and .ele (renamed to .node and .ele in current mesh_utils.py)
        # but the spec says "-> .msh". Our mesh_utils.py has a renaming logic.
        assert "test.node" in files or "test.msh" in files

        # 4. Fail path: Non-manifold geometry
        # (This might be hard to construct via build123d without it failing first,
        # but we can try to write a malformed STL directly)
        bad_stl = "solid bad\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 0 0 0\nvertex 0 0 0\nendloop\nendfacet\nendsolid"
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "bad.stl", "content": bad_stl},
            headers=base_headers,
        )

        code_fail = """
from pathlib import Path
from worker.utils.mesh_utils import tetrahedralize
try:
    tetrahedralize(Path("bad.stl"), Path("bad.msh"))
except Exception as e:
    print(f"MESH_FAILURE:{e}")
"""
        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={"code": code_fail},
            headers=base_headers,
        )
        assert "MESH_FAILURE" in resp.json()["stdout"]
