import unittest
from pathlib import Path

from src.environment.sandbox import PodmanSandbox


class TestSandbox(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workspace = Path("tests/output/sandbox_test").resolve()
        cls.workspace.mkdir(parents=True, exist_ok=True)
        cls.sandbox = PodmanSandbox(str(cls.workspace))

    def test_run_script_basic(self):
        script_name = "hello.py"
        (self.workspace / script_name).write_text("print('Hello from Sandbox')")

        stdout, stderr, rc = self.sandbox.run_script(script_name)
        self.assertEqual(rc, 0, f"Error: {stderr}")
        self.assertIn("Hello from Sandbox", stdout)

    def test_network_isolation(self):
        # Try to use python to check networking
        # We expect a failure when trying to connect to 8.8.8.8
        script = "import socket; s = socket.socket(); s.settimeout(1); print(s.connect_ex(('8.8.8.8', 53)))"
        script_name = "net_test.py"
        (self.workspace / script_name).write_text(script)

        stdout, stderr, rc = self.sandbox.run_script(script_name)
        # returncode from connect_ex should be non-zero (errno) if blocked
        # Or connection timeout
        self.assertNotEqual(stdout.strip(), "0")

    def test_resource_limits(self):
        # Infinite loop with short timeout
        script_name = "loop.py"
        (self.workspace / script_name).write_text("while True: pass")

        stdout, stderr, rc = self.sandbox.run_script(script_name, timeout=3)
        # podman kill / timeout usually results in code 124 or similar
        self.assertIn(rc, [124, 137, 1])  # 124 is timeout command, 137 is SIGKILL

    def test_mount_src(self):
        # Check if we can import our own code from src (since we mounted it)
        script_name = "import_test.py"
        (self.workspace / script_name).write_text(
            "from src.cots.core import Part; print('Import Success')"
        )

        stdout, stderr, rc = self.sandbox.run_script(script_name, mount_src=True)
        self.assertEqual(rc, 0, f"Error: {stderr}")
        self.assertIn("Import Success", stdout)

    def test_persistent_session(self):
        session_id = "test-session-123"
        try:
            # Start session
            success = self.sandbox.start_session(session_id)
            self.assertTrue(success)

            # Exec command
            stdout, stderr, rc = self.sandbox.exec_command(
                session_id, ["echo", "hello-persist"]
            )
            self.assertEqual(rc, 0)
            self.assertIn("hello-persist", stdout)

            # Write a file and check persistence
            (self.workspace / "persist_test.txt").write_text("file-content")
            stdout, stderr, rc = self.sandbox.exec_command(
                session_id, ["cat", "persist_test.txt"]
            )
            self.assertEqual(rc, 0)
            self.assertIn("file-content", stdout)
        finally:
            self.sandbox.stop_session(session_id)


if __name__ == "__main__":
    unittest.main()
