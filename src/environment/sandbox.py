import subprocess
import os
import logging
from typing import Tuple, List, Optional

logger = logging.getLogger(__name__)


class PodmanSandbox:
    """
    Provides isolated execution of Python code using Podman containers.
    """

    def __init__(self, workspace_dir: str, image: str = "cad-sandbox"):
        self.workspace_dir = os.path.abspath(workspace_dir)
        self.image = image
        # Ensure workspace exists
        os.makedirs(self.workspace_dir, exist_ok=True)

    def run_script(
        self,
        script_name: str,
        timeout: int = 30,
        memory_limit: str = "1g",
        cpu_quota: int = 50000,
        network: str = "none",
        mount_src: bool = False,
        extra_mounts: Optional[List[Tuple[str, str]]] = None,
    ) -> Tuple[str, str, int]:
        """
        Runs a script inside the sandbox.

        Args:
            script_name: Name of the script in the workspace.
            timeout: Maximum execution time in seconds.
            memory_limit: RAM limit (e.g., '1g').
            cpu_quota: CPU time quota.
            network: Network mode ('none' by default).
            mount_src: Whether to mount the project's src directory.
            extra_mounts: List of (host_path, container_path) tuples.

        Returns:
            (stdout, stderr, return_code)
        """
        container_workspace = "/workspace"

        # We use --rm to remove the container after exit
        # We mount the workspace with :Z for SELinux context labeling if on Fedora/RHEL
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
            project_root = os.path.abspath(
                os.path.join(os.path.dirname(__file__), "../..")
            )
            src_path = os.path.join(project_root, "src")
            # Mount src to /app/src and set PYTHONPATH to /app
            cmd.extend(["-v", f"{src_path}:/app/src:ro", "-e", "PYTHONPATH=/app"])

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
            logger.info(f"Running sandboxed script: {script_name}")
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout
            )
            return result.stdout, result.stderr, result.returncode
        except subprocess.TimeoutExpired:
            logger.warning(f"Sandbox timeout for {script_name}")
            return "", "Error: Execution timed out (Sandbox limit exceeded).", 124
        except Exception as e:
            logger.error(f"Sandbox error: {str(e)}")
            return "", f"Error running Podman: {str(e)}", 1

    def run_command(self, python_code: str, timeout: int = 30) -> Tuple[str, str, int]:
        """
        Runs a snippet of Python code directly.
        """
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
