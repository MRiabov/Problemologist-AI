import os
import time

import httpx
import pytest

from shared.workers.schema import (
    ExecuteRequest,
    ExecuteResponse,
    FsFileEntry,
    ListFilesRequest,
    WriteFileRequest,
)

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")


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
    async with httpx.AsyncClient(timeout=300.0) as client:
        # 1. Write a valid STL script
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
        write_req = WriteFileRequest(
            path="box.py", content=script_content, overwrite=True
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_req.model_dump(mode="json"),
            headers=base_headers,
        )

        # 2. Trigger tetrahedralization
        code = """
from pathlib import Path
from worker_heavy.utils.mesh_utils import tetrahedralize
from build123d import Box, export_stl

part = Box(1, 1, 1)
export_stl(part, "test.stl")
msh_path = tetrahedralize(Path("test.stl"), Path("test.msh"))
assert msh_path.exists(), f"Mesh file not created at {msh_path}"
"""
        exec_req = ExecuteRequest(code=code, timeout=60)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=exec_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=60.0,
        )
        assert resp.status_code == 200, f"Execution failed: {resp.text}"
        data = ExecuteResponse.model_validate(resp.json())
        assert data.exit_code == 0, (
            f"Meshing script failed: {data.stdout} {data.stderr}"
        )

        # 3. Verify files exist in session
        ls_req = ListFilesRequest(path=".")
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ls_req.model_dump(mode="json"),
            headers=base_headers,
        )
        fs_entries = [FsFileEntry.model_validate(e) for e in ls_resp.json()]
        file_names = [e.name for e in fs_entries]
        assert "test.node" in file_names or "test.msh" in file_names, (
            f"Missing mesh files. Found: {file_names}"
        )

        # 4. Fail path: Non-manifold geometry
        bad_stl = "solid bad\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 0 0 0\nvertex 0 0 0\nendloop\nendfacet\nendsolid"
        write_bad_stl = WriteFileRequest(
            path="bad.stl", content=bad_stl, overwrite=True
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_bad_stl.model_dump(mode="json"),
            headers=base_headers,
        )

        code_fail = """
from pathlib import Path
from worker_heavy.utils.mesh_utils import tetrahedralize
tetrahedralize(Path("bad.stl"), Path("bad.msh"))
"""
        exec_fail_req = ExecuteRequest(code=code_fail, timeout=60)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=exec_fail_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=60.0,
        )
        data_fail = ExecuteResponse.model_validate(resp.json())
        assert data_fail.exit_code != 0, "Expected meshing to fail for malformed STL"
