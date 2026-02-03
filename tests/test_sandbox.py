"""Sandbox tests refactored for parallel execution using tmp_path fixture."""

import pytest

from src.environment.sandbox import PodmanSandbox


@pytest.fixture
def sandbox(tmp_path):
    """Create an isolated sandbox using pytest's tmp_path for parallel safety."""
    workspace = tmp_path / "sandbox_test"
    workspace.mkdir()
    return PodmanSandbox(str(workspace)), workspace


def test_run_script_basic(sandbox):
    sandbox_instance, workspace = sandbox
    script_name = "hello.py"
    (workspace / script_name).write_text("print('Hello from Sandbox')")

    stdout, stderr, rc = sandbox_instance.run_script(script_name)
    assert rc == 0, f"Error: {stderr}"
    assert "Hello from Sandbox" in stdout


def test_network_isolation(sandbox):
    sandbox_instance, workspace = sandbox
    script = "import socket; s = socket.socket(); s.settimeout(1); print(s.connect_ex(('8.8.8.8', 53)))"
    script_name = "net_test.py"
    (workspace / script_name).write_text(script)

    stdout, stderr, rc = sandbox_instance.run_script(script_name)
    # returncode from connect_ex should be non-zero (errno) if blocked
    assert stdout.strip() != "0"


def test_resource_limits(sandbox):
    sandbox_instance, workspace = sandbox
    script_name = "loop.py"
    (workspace / script_name).write_text("while True: pass")

    stdout, stderr, rc = sandbox_instance.run_script(script_name, timeout=3)
    # podman kill / timeout usually results in code 124 or similar
    assert rc in [124, 137, 1]  # 124 is timeout command, 137 is SIGKILL


def test_mount_src(sandbox):
    sandbox_instance, workspace = sandbox
    script_name = "import_test.py"
    (workspace / script_name).write_text(
        "from src.cots.core import Part; print('Import Success')"
    )

    stdout, stderr, rc = sandbox_instance.run_script(script_name, mount_src=True)
    assert rc == 0, f"Error: {stderr}"
    assert "Import Success" in stdout


def test_persistent_session(sandbox, request):
    sandbox_instance, workspace = sandbox
    # Use test node id for unique session name to avoid conflicts
    session_id = f"test-session-{request.node.name}"
    try:
        # Start session
        success = sandbox_instance.start_session(session_id)
        assert success

        # Exec command
        stdout, stderr, rc = sandbox_instance.exec_command(
            session_id, ["echo", "hello-persist"]
        )
        assert rc == 0
        assert "hello-persist" in stdout

        # Write a file and check persistence
        (workspace / "persist_test.txt").write_text("file-content")
        stdout, stderr, rc = sandbox_instance.exec_command(
            session_id, ["cat", "persist_test.txt"]
        )
        assert rc == 0
        assert "file-content" in stdout
    finally:
        sandbox_instance.stop_session(session_id)
