import os
import unittest

from src.environment.sandbox import PodmanSandbox


class TestSandbox(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workspace = os.path.abspath("workspace_test")
        os.makedirs(cls.workspace, exist_ok=True)
        cls.sandbox = PodmanSandbox(cls.workspace)

    def test_run_script_basic(self):
        script_name = "hello.py"
        with open(os.path.join(self.workspace, script_name), "w") as f:
            f.write("print('Hello from Sandbox')")

        stdout, stderr, rc = self.sandbox.run_script(script_name)
        self.assertEqual(rc, 0, f"Error: {stderr}")
        self.assertIn("Hello from Sandbox", stdout)

    def test_network_isolation(self):
        # Try to use python to check networking
        # We expect a failure when trying to connect to 8.8.8.8
        script = "import socket; s = socket.socket(); s.settimeout(1); print(s.connect_ex(('8.8.8.8', 53)))"
        script_name = "net_test.py"
        with open(os.path.join(self.workspace, script_name), "w") as f:
            f.write(script)

        stdout, stderr, rc = self.sandbox.run_script(script_name)
        # returncode from connect_ex should be non-zero (errno) if blocked
        # Or connection timeout
        self.assertNotEqual(stdout.strip(), "0")

    def test_resource_limits(self):
        # Infinite loop with short timeout
        script_name = "loop.py"
        with open(os.path.join(self.workspace, script_name), "w") as f:
            f.write("while True: pass")

        stdout, stderr, rc = self.sandbox.run_script(script_name, timeout=3)
        # podman kill / timeout usually results in code 124 or similar
        self.assertIn(rc, [124, 137, 1])  # 124 is timeout command, 137 is SIGKILL

    def test_mount_src(self):
        # Check if we can import our own code from src (since we mounted it)
        script_name = "import_test.py"
        with open(os.path.join(self.workspace, script_name), "w") as f:
            f.write("from src.compiler import geometry; print('Import Success')")

        stdout, stderr, rc = self.sandbox.run_script(script_name, mount_src=True)
        self.assertEqual(rc, 0, f"Error: {stderr}")
        self.assertIn("Import Success", stdout)


if __name__ == "__main__":
    unittest.main()
