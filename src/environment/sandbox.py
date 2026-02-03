import logging
import subprocess
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


class PodmanSandbox:
    """
    Provides isolated execution of Python code using Podman containers.
    Supports both transient (run) and persistent (exec) modes.
    """

    def __init__(self, workspace_dir: str, image: str = "problemologist-sandbox"):
        self.workspace_dir = str(Path(workspace_dir).resolve())
        self.image = image
        # Ensure workspace exists
        Path(self.workspace_dir).mkdir(parents=True, exist_ok=True)
        self._active_sessions: set[str] = set()

    def start_session(
        self,
        session_id: str,
        memory_limit: str = "1g",
        cpu_quota: int = 50000,
        network: str = "none",
        mount_src: bool = True,
    ) -> bool:
        """
        Starts a persistent container session.
        """
        if session_id in self._active_sessions:
            logger.warning(f"Session {session_id} already exists.")
            return True

        container_workspace = "/workspace"
        project_root = Path(__file__).resolve().parent.parent.parent
        src_path = project_root / "src"
        skills_path = project_root / ".agent" / "skills"

        cmd = [
            "podman",
            "run",
            "-d",
            "--rm",
            "--name",
            session_id,
            "--network",
            network,
            "--memory",
            memory_limit,
            "--cpu-quota",
            str(cpu_quota),
            "-v",
            f"{self.workspace_dir}:{container_workspace}:Z",
        ]

        if mount_src:
            # Mount src to /app/src and set PYTHONPATH to /app
            # Also mount config to /app/config
            config_path = project_root / "config"
            cmd.extend(
                [
                    "-v",
                    f"{src_path}:/app/src:ro",
                    "-v",
                    f"{config_path}:/app/config:ro",
                    "-v",
                    f"{skills_path}:/workspace/docs/skills:rw",  # Map skills to docs/skills as per plan
                    "-e",
                    "PYTHONPATH=/app:/workspace",
                ]
            )

        cmd.extend([self.image, "sleep", "infinity"])

        try:
            logger.info(f"Starting persistent sandbox session: {session_id}")
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            self._active_sessions.add(session_id)
            return True
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to start sandbox session {session_id}: {e.stderr}")
            return False

    def stop_session(self, session_id: str) -> bool:
        """
        Stops and removes the persistent container session.
        """
        if session_id not in self._active_sessions:
            return True

        try:
            logger.info(f"Stopping sandbox session: {session_id}")
            subprocess.run(
                ["podman", "kill", session_id], check=True, capture_output=True
            )
            self._active_sessions.remove(session_id)
            return True
        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to stop sandbox session {session_id}: {e.stderr}")
            return False

    def exec_command(
        self,
        session_id: str,
        cmd_args: list[str],
        workdir: str = "/workspace",
        timeout: int = 30,
    ) -> tuple[str, str, int]:
        """
        Executes a command inside an active session.
        """
        if session_id not in self._active_sessions:
            return "", f"Error: Session {session_id} is not active.", 1

        exec_cmd = [
            "podman",
            "exec",
            "--workdir",
            workdir,
            session_id,
        ] + cmd_args

        try:
            result = subprocess.run(
                exec_cmd, capture_output=True, text=True, timeout=timeout
            )
            return result.stdout, result.stderr, result.returncode
        except subprocess.TimeoutExpired:
            return "", "Error: Command timed out.", 124
        except Exception as e:
            return "", str(e), 1

    def run_script(
        self,
        script_name: str,
        timeout: int = 30,
        memory_limit: str = "1g",
        cpu_quota: int = 50000,
        network: str = "none",
        mount_src: bool = False,
        extra_mounts: list[tuple[str, str]] | None = None,
        session_id: Optional[str] = None,
    ) -> tuple[str, str, int]:
        """
        Runs a script. If session_id is provided and active, uses exec.
        Otherwise, uses 'podman run' (transient).
        """
        if session_id and session_id in self._active_sessions:
            return self.exec_command(
                session_id, ["python", script_name], timeout=timeout
            )

        # Fallback to transient mode (unchanged logic but cleaned up)
        container_workspace = "/workspace"
        cmd = [
            "podman",
            "run",
            "--rm",
            "--network",
            network,
            "--memory",
            memory_limit,
            "--cpu-quota",
            str(cpu_quota),
            "-v",
            f"{self.workspace_dir}:{container_workspace}:Z",
        ]

        if extra_mounts:
            for host_path, container_path in extra_mounts:
                cmd.extend(["-v", f"{host_path}:{container_path}:ro"])

        if mount_src:
            project_root = Path(__file__).resolve().parent.parent.parent
            src_path = project_root / "src"
            config_path = project_root / "config"
            cmd.extend(
                [
                    "-v",
                    f"{src_path}:/app/src:ro",
                    "-v",
                    f"{config_path}:/app/config:ro",
                    "-e",
                    "PYTHONPATH=/app",
                ]
            )

        cmd.extend(
            [
                "--workdir",
                container_workspace,
                self.image,
                "python",
                script_name,
            ]
        )

        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout
            )
            return result.stdout, result.stderr, result.returncode
        except subprocess.TimeoutExpired:
            return "", "Error: Execution timed out.", 124
        except Exception as e:
            return "", str(e), 1

    def run_command(
        self, python_code: str, timeout: int = 30, session_id: Optional[str] = None
    ) -> tuple[str, str, int]:
        """
        Runs a snippet of Python code.
        """
        if session_id and session_id in self._active_sessions:
            return self.exec_command(
                session_id, ["python", "-c", python_code], timeout=timeout
            )

        # Fallback to transient mode
        container_workspace = "/workspace"
        cmd = [
            "podman",
            "run",
            "--rm",
            "--network",
            "none",
            "-v",
            f"{self.workspace_dir}:{container_workspace}:Z",
            "--workdir",
            container_workspace,
            self.image,
            "python",
            "-c",
            python_code,
        ]

        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout
            )
            return result.stdout, result.stderr, result.returncode
        except Exception as e:
            return "", str(e), 1
