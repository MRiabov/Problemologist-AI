import logging
import socket
import subprocess
import time
from pathlib import Path
from typing import Optional

import httpx

from src.environment.models import (
    CommandRequest,
    CommandResponse,
    ScriptRequest,
)

logger = logging.getLogger(__name__)


class PodmanSandbox:
    """
    Provides isolated execution of Python code using Podman containers.
    Supports both transient (run) and persistent (exec) modes via HTTP agent.
    """

    def __init__(self, workspace_dir: str, image: str = "problemologist-sandbox"):
        self.workspace_dir = str(Path(workspace_dir).resolve())
        self.image = image
        # Ensure workspace exists
        Path(self.workspace_dir).mkdir(parents=True, exist_ok=True)
        # Map session_id -> port
        self._active_sessions: dict[str, int] = {}

    def _find_free_port(self) -> int:
        """Finds a free port on the host."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(("", 0))
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            return s.getsockname()[1]

    def _wait_for_agent(self, port: int, timeout: int = 10) -> bool:
        """Waits for the container agent to be ready."""
        url = f"http://localhost:{port}/health"
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                resp = httpx.get(url, timeout=1)
                if resp.status_code == 200:
                    return True
            except httpx.RequestError:
                pass
            time.sleep(0.5)
        return False

    def start_session(
        self,
        session_id: str,
        memory_limit: str = "1g",
        cpu_quota: int = 50000,
        network: str = "host",  # Use host networking or map ports? Prefer port map with bridge.
        mount_src: bool = True,
    ) -> bool:
        """
        Starts a persistent container session with the agent API.
        """
        if session_id in self._active_sessions:
            logger.warning(f"Session {session_id} already exists.")
            return True

        container_workspace = "/workspace"
        project_root = Path(__file__).resolve().parent.parent.parent
        src_path = project_root / "src"
        skills_path = project_root / ".agent" / "skills"

        # Find a free port
        host_port = self._find_free_port()

        # Update network mode. "none" blocks API. Use "slirp4netns" or "bridge" (default).
        # Should we use "none" with port forwarding?
        # Actually, to access host from container we might need network, but we access container from host.
        # So standard bridge network with -p is fine.
        if network == "none":
            # If specifically requested none, we can't use HTTP unless we use unix sockets or similar?
            # Or podman uses slirp4netns by default for rootless.
            # Let's switch default to "bridge" or just remove strict "none" if we need HTTP.
            # But the user might want isolation.
            # If we map ports, podman handles it.
            pass

        cmd = [
            "podman",
            "run",
            "-d",
            "--rm",
            "--name",
            session_id,
            # "--network", network, # Removing network restriction to allow port mapping working reliably?
            # Or keep it if it works with -p.
            # "-p", f"{host_port}:8000",
            f"-p{host_port}:8000",
            "--memory",
            memory_limit,
            "--cpu-quota",
            str(cpu_quota),
            "-v",
            f"{self.workspace_dir}:{container_workspace}:Z",
        ]

        if mount_src:
            config_path = project_root / "config"
            cmd.extend(
                [
                    "-v",
                    f"{src_path}:/app/src:ro",
                    "-v",
                    f"{config_path}:/app/config:ro",
                    "-v",
                    f"{skills_path}:/workspace/docs/skills:rw",
                    "-e",
                    "PYTHONPATH=/app:/workspace",
                ]
            )

        # Command to run the agent
        agent_script = "/app/src/assets/scripts/container_agent.py"

        # We assume dependencies are in the image now.
        startup_cmd = f"python {agent_script}"

        cmd.extend([self.image, "sh", "-c", startup_cmd])

        try:
            logger.info(
                f"Starting persistent sandbox session: {session_id} on port {host_port}"
            )
            subprocess.run(cmd, check=True, capture_output=True, text=True)

            # Wait for agent
            if self._wait_for_agent(host_port):
                self._active_sessions[session_id] = host_port
                return True

            logger.error(f"Agent failed to start in session {session_id}.")
            # Log container logs
            try:
                logs = subprocess.run(
                    ["podman", "logs", session_id], capture_output=True, text=True
                )
                logger.error(f"Container Logs:\n{logs.stdout}{logs.stderr}")
            except Exception:
                logger.error("Could not retrieve container logs.")

            # KIll it
            self.stop_session(session_id)
            return False

        except subprocess.CalledProcessError as e:
            logger.error(f"Failed to start sandbox session {session_id}: {e.stderr}")
            return False

    def stop_session(self, session_id: str) -> bool:
        """
        Stops and removes the persistent container session.
        """
        if session_id not in self._active_sessions:
            # Check if container exists anyway (orphan)
            pass

        try:
            logger.info(f"Stopping sandbox session: {session_id}")
            subprocess.run(
                ["podman", "kill", session_id], check=False, capture_output=True
            )
            subprocess.run(
                ["podman", "rm", "-f", session_id], check=False, capture_output=True
            )
            if session_id in self._active_sessions:
                del self._active_sessions[session_id]
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
        Executes a command inside an active session via HTTP agent.
        """
        port = self._active_sessions.get(session_id)
        if not port:
            return "", f"Error: Session {session_id} is not active.", 1

        # Reconstruct command string from args
        # cmd_args is list like ["ls", "-la"] or ["python", "script.py"]
        # We join them safely? Or pass as list?
        # CommandRequest expects a string "command".
        import shlex

        command_str = shlex.join(cmd_args)

        req = CommandRequest(command=command_str, workdir=workdir, timeout=timeout)
        try:
            url = f"http://localhost:{port}/run_command"
            resp = httpx.post(url, json=req.model_dump(), timeout=timeout + 5)
            if resp.status_code != 200:
                return "", f"Agent Error: {resp.status_code} {resp.text}", 1

            data = CommandResponse(**resp.json())
            return data.stdout, data.stderr, data.return_code

        except httpx.TimeoutException:
            return "", "Error: Command timed out (agent).", 124
        except Exception as e:
            return "", f"Communication Error: {e}", 1

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
        Runs a script. If session_id is active, uses agent /run_script.
        Otherwise, falls back to transient.
        """
        if session_id and session_id in self._active_sessions:
            port = self._active_sessions[session_id]
            req = ScriptRequest(script_name=script_name, timeout=timeout)
            try:
                url = f"http://localhost:{port}/run_script"
                resp = httpx.post(url, json=req.model_dump(), timeout=timeout + 5)
                if resp.status_code != 200:
                    return "", f"Agent Error: {resp.status_code} {resp.text}", 1

                data = CommandResponse(**resp.json())
                return data.stdout, data.stderr, data.return_code
            except Exception as e:
                return "", f"Communication Error: {e}", 1

        # Fallback to transient mode (unchanged logic)
        container_workspace = "/workspace"
        cmd = [
            "podman",
            "run",
            "--rm",
            # "--network", network, # Keep network restriction for transient?
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
            # Use run_command with python -c
            # Be careful with quoting. pass as list to exec_command?
            # exec_command takes list.
            return self.exec_command(
                session_id, ["python", "-c", python_code], timeout=timeout
            )

        # Fallback to transient mode
        container_workspace = "/workspace"
        cmd = [
            "podman",
            "run",
            "--rm",
            # "--network", "none",
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
