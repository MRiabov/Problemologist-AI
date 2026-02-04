import logging
import time

import pytest
from src.environment.sandbox import PodmanSandbox

logger = logging.getLogger(__name__)


@pytest.mark.integration
def test_container_bridge_e2e(tmp_path):
    """
    Verifies that PodmanSandbox can start a persistent session with the HTTP agent
    and execute commands/scripts via the HTTP API.
    """
    workspace = tmp_path / "test_http_sandbox"
    workspace.mkdir(parents=True, exist_ok=True)

    # Create a dummy script in workspace
    (workspace / "test_script.py").write_text("print('Hello from script!')")

    sandbox = PodmanSandbox(str(workspace))
    session_id = f"test-agent-{int(time.time())}"

    try:
        logger.info("Starting session...")
        success = sandbox.start_session(session_id)
        assert success, "Failed to start session"

        # start_session already waits for agent

        logger.info("Testing run_command (HTTP)...")
        stdout, stderr, rc = sandbox.exec_command(
            session_id, ["echo", "Hello from agent!"]
        )
        logger.info(f"Result: stdout='{stdout.strip()}', stderr='{stderr}', rc={rc}")

        assert rc == 0
        assert "Hello from agent!" in stdout

        logger.info("Testing run_script (HTTP)...")
        # Script is in workspace, which is mounted to /workspace
        stdout, stderr, rc = sandbox.run_script("test_script.py", session_id=session_id)
        logger.info(f"Result: stdout='{stdout.strip()}', stderr='{stderr}', rc={rc}")

        assert rc == 0
        assert "Hello from script!" in stdout

        logger.info("Testing run_command fallback...")
        # sandbox.run_command uses python -c via exec_command if session active
        stdout, stderr, rc = sandbox.run_command(
            "print(100 + 200)", session_id=session_id
        )
        logger.info(f"Result: stdout='{stdout.strip()}', stderr='{stderr}', rc={rc}")

        assert rc == 0
        assert "300" in stdout

    finally:
        logger.info("Stopping session...")
        sandbox.stop_session(session_id)
