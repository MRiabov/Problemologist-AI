import os
import shutil
import tempfile
import uuid
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import HumanMessage

import controller.agent.nodes.base
import controller.middleware.remote_fs
import controller.observability.middleware_helper

# NUCLEAR OPTION: Mock observability at the module level before anything else imports it
import controller.observability.tracing

mock_async = AsyncMock()
controller.observability.tracing.record_worker_events = mock_async
controller.observability.tracing.sync_asset = mock_async
controller.observability.middleware_helper.record_events = mock_async
controller.observability.middleware_helper.broadcast_file_update = mock_async
controller.middleware.remote_fs.record_events = mock_async
controller.middleware.remote_fs.broadcast_file_update = mock_async
controller.middleware.remote_fs.record_simulation_result = mock_async
controller.middleware.remote_fs.sync_file_asset = mock_async

# Fix NameError for logger in base.py
controller.agent.nodes.base.logger = MagicMock()

from controller.agent.benchmark.models import GenerationSession, SessionStatus
from controller.agent.benchmark.nodes import BenchmarkCoderNode, BenchmarkGeneratorState
from controller.agent.mock_llm import MockDSPyLM
from controller.agent.nodes.base import SharedNodeContext
from controller.clients.worker import WorkerClient
from shared.simulation.schemas import RandomizationStrategy


@pytest.fixture
def temp_session_dir():
    path = tempfile.mkdtemp()
    yield Path(path)
    shutil.rmtree(path)


@pytest.mark.asyncio
async def test_benchmark_coder_with_mock_llm(temp_session_dir):
    """
    Test BenchmarkCoderNode using MockDSPyLM and a local filesystem.
    """
    session_id = str(uuid.uuid4())

    # 1. Setup local mock worker environment
    mock_client = MagicMock(spec=WorkerClient)
    mock_client.session_id = session_id

    async def mock_write(path, content, overwrite=True):
        full_path = temp_session_dir / path.lstrip("/")
        full_path.parent.mkdir(parents=True, exist_ok=True)
        full_path.write_text(content)
        return True

    async def mock_read(path):
        full_path = temp_session_dir / path.lstrip("/")
        if not full_path.exists():
            if "objectives.yaml" in path:
                return "objectives: {}"
            return "empty"
        return full_path.read_text()

    async def mock_exists(path):
        return (temp_session_dir / path.lstrip("/")).exists()

    async def mock_execute(code, timeout=30):
        original_cwd = os.getcwd()
        os.chdir(temp_session_dir)
        try:
            # We don't need real build123d here, just the side effects
            # (writing files)
            namespace = {"__name__": "__main__"}
            exec(code, namespace)
            return MagicMock(
                stdout="Executed successfully",
                stderr="",
                exit_code=0,
                model_dump=lambda: {"stdout": "Executed", "stderr": "", "exit_code": 0},
            )
        except Exception as e:
            return MagicMock(
                stdout="",
                stderr=str(e),
                exit_code=1,
                model_dump=lambda: {"stdout": "", "stderr": str(e), "exit_code": 1},
            )
        finally:
            os.chdir(original_cwd)

    mock_client.write_file = mock_write
    mock_client.read_file = mock_read
    mock_client.exists = mock_exists
    mock_client.execute_python = mock_execute
    mock_client.validate = AsyncMock(return_value=MagicMock(success=True))
    mock_client.simulate = AsyncMock(return_value=MagicMock(success=True, artifacts={}))
    mock_client.list_files = AsyncMock(return_value=[])

    # 2. Setup Mock LLM
    with patch("controller.agent.nodes.base.settings") as mock_settings:
        mock_settings.is_integration_test = True
        mock_settings.worker_heavy_url = "http://heavy"
        mock_settings.spec_001_api_url = "http://light"
        mock_settings.database_url = (
            "postgresql+asyncpg://mock:mock@localhost:5432/mock"
        )

        # Use our own MockDSPyLM instance
        mock_lm = MockDSPyLM(session_id="benchmark")

        with (
            patch("controller.agent.nodes.base.WorkerClient", return_value=mock_client),
            patch("controller.agent.mock_llm.MockDSPyLM", return_value=mock_lm),
            patch("controller.observability.tracing.get_sessionmaker") as mock_sm,
            patch("controller.agent.tools.record_worker_events", new=mock_async),
        ):
            # Totally block DB connection attempts
            mock_sm.side_effect = RuntimeError("DB connection blocked in unit test")

            ctx = SharedNodeContext.create(
                worker_light_url="http://light", session_id=session_id
            )
            node = BenchmarkCoderNode(context=ctx)

            # 3. Setup Initial State
            session = GenerationSession(
                session_id=uuid.UUID(session_id),
                prompt="Create a simple ball benchmark.",
                status=SessionStatus.PLANNING,
                validation_logs=[],
            )

            state = BenchmarkGeneratorState(
                session=session,
                plan=RandomizationStrategy(theme="ball_move", reasoning="test"),
                messages=[HumanMessage(content="Start")],
                journal="",
            )

            # 4. Execute Node
            result_state = await node(state)

            # 5. Assertions
            assert (temp_session_dir / "script.py").exists()
            script_content = (temp_session_dir / "script.py").read_text()
            assert "def build():" in script_content
            assert (temp_session_dir / "objectives.yaml").exists()
            assert (temp_session_dir / "plan.md").exists()
            assert (temp_session_dir / "todo.md").exists()

            assert any(
                "Work summary" in m.content
                for m in result_state.messages
                if hasattr(m, "content")
            )
