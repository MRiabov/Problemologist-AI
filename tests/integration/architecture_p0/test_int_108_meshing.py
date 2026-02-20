import os
import time

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


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
    INT-108: Verify STL -> Gmsh/TetGen -> .msh flow.
    """
    async with httpx.AsyncClient() as client:
        # 1. Trigger tetrahedralization via /runtime/execute to test the internal tool
        # This simulates how a subagent or internal utility would use Gmsh.
        code = """
from pathlib import Path
from worker_heavy.utils.mesh_utils import tetrahedralize
from build123d import Box, export_stl

# Create a simple geometry
part = Box(1, 1, 1)
export_stl(part, "test.stl")

# Tetrahedralize using default method (Gmsh)
msh_path = tetrahedralize(Path("test.stl"), Path("test.msh"))
assert msh_path.exists(), f"Mesh file not created at {msh_path}"
assert msh_path.stat().st_size > 0, "Mesh file is empty"
"""
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json={"code": code},
            headers=base_headers,
            timeout=60.0,
        )
        assert resp.status_code == 200, f"Execution failed: {resp.text}"
        data = resp.json()
        assert data["exit_code"] == 0, (
            f"Meshing script failed: {data['stdout']} {data['stderr']}"
        )

        # 2. Verify files exist in session
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls", json={"path": "."}, headers=base_headers
        )
        files = [f["name"] for f in ls_resp.json()]
        # Verify both input STL and output MSH exist
        assert "test.stl" in files, f"Input STL missing. Found: {files}"
        assert "test.msh" in files, f"Output MSH missing. Found: {files}"

        # 4. Fail path: Non-manifold geometry
        # (This might be hard to construct via build123d without it failing first,
        # but we can try to write a malformed STL directly)
        bad_stl = "solid bad\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 0 0 0\nvertex 0 0 0\nendloop\nendfacet\nendsolid"
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "bad.stl", "content": bad_stl},
            headers=base_headers,
        )

        code_fail = """
from pathlib import Path
from worker_heavy.utils.mesh_utils import tetrahedralize
tetrahedralize(Path("bad.stl"), Path("bad.msh"))
"""
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json={"code": code_fail},
            headers=base_headers,
            timeout=60.0,
        )
        assert resp.json()["exit_code"] != 0, (
            "Expected meshing to fail for malformed STL"
        )
