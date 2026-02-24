import pytest
import subprocess
import os
import sys
from unittest.mock import patch, MagicMock

# Add dev/automations to path so we can import the scripts
PROJ_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
AUTOMATIONS_DIR = os.path.join(PROJ_ROOT, "dev/automations")
sys.path.append(AUTOMATIONS_DIR)

# Import the modules to test
try:
    import nexus_orchestrator
    import overnight_orchestrator
except ImportError:
    pass


class TestNexusOrchestrator:
    def test_parse_failures_regex(self):
        """Test that the regex correctly identifies failing tests from stdout."""
        if "nexus_orchestrator" not in sys.modules:
            pytest.skip("nexus_orchestrator module not found")

        sample_output = """
        tests/integration/test_foo.py::test_bar PASSED
        tests/integration/test_baz.py::test_qux FAILED
        FAILED tests/integration/test_worker_concurrency.py::test_worker_concurrency - AssertionError
        """
        failures = nexus_orchestrator.parse_failures(sample_output)
        # The regex actually extracts the full test ID (path::name) in the current implementation
        assert (
            "tests/integration/test_worker_concurrency.py::test_worker_concurrency"
            in failures
        )
        # Note: The current regex might be specific about the line format.
        # Let's verify what the actual regex expects based on the source code.

    @patch("subprocess.run")
    def test_check_process(self, mock_run):
        """Test checking for running processes."""
        if "nexus_orchestrator" not in sys.modules:
            pytest.skip("nexus_orchestrator module not found")

        # Mock process found
        mock_run.return_value = MagicMock(
            stdout="12345 python script.py\n", returncode=0
        )
        assert nexus_orchestrator.check_process("script.py") is True

        # Mock process not found
        mock_run.return_value = MagicMock(stdout="", returncode=1)
        assert nexus_orchestrator.check_process("nonexistent") is False


class TestOvernightOrchestrator:
    def test_parse_failures_xml_fallback(self):
        """Test that we fallback gracefully if XML is missing."""
        if "overnight_orchestrator" not in sys.modules:
            pytest.skip("overnight_orchestrator module not found")

        # Ensure we don't actually read a real file
        with patch("os.path.exists", return_value=False):
            failures = overnight_orchestrator.parse_failures_xml("fake.xml")
            assert failures == []

    def test_parse_failures_stdout(self):
        """Test stdout parsing logic."""
        if "overnight_orchestrator" not in sys.modules:
            pytest.skip("overnight_orchestrator module not found")

        sample_output = """
        FAILED tests/integration/test_api.py::test_endpoint - 404 != 200
        """
        # The function returns a list of dicts with 'id' and 'details'
        failures = overnight_orchestrator.parse_failures(sample_output)
        assert len(failures) == 1
        # The regex captures the full test ID
        assert failures[0]["id"] == "tests/integration/test_api.py::test_endpoint"
