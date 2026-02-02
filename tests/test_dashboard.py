import pytest
from streamlit.testing.v1 import AppTest
import sys
import os

# Ensure project root is in sys.path for the test runner
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
if root_path not in sys.path:
    sys.path.insert(0, root_path)


def test_dashboard_smoke():
    """Basic smoke test to ensure the dashboard starts without crashing."""
    at = AppTest.from_file("src/dashboard/main.py")
    at.run(timeout=30)
    assert not at.exception
    assert at.sidebar.header[0].body == "Dashboard Controls"


def test_mode_switching():
    """Verify that switching modes works and doesn't cause duplicate element errors."""
    at = AppTest.from_file("src/dashboard/main.py")
    at.run(timeout=30)

    # Locate radio and switch to Benchmark Generator
    radio = at.sidebar.radio(key="dashboard_mode_selector")
    radio.set_value("Benchmark Generator").run(timeout=30)

    assert not at.exception
    assert "Interactive Benchmark Generator" in at.header[0].body


def test_benchmark_generator_input():
    """Verify the first stage of benchmark generator."""
    at = AppTest.from_file("src/dashboard/main.py")
    at.run(timeout=30)

    at.sidebar.radio(key="dashboard_mode_selector").set_value(
        "Benchmark Generator"
    ).run(timeout=30)

    assert at.subheader[0].body == "1. Describe the Benchmark"
    assert at.button[0].label == "Generate Plan"
