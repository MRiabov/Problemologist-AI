import json
from unittest.mock import MagicMock, patch

from src.generators.benchmark.linter import run_linter


def test_run_linter_valid():
    code = """
def build(seed=0, scale=(1,1,1)):
    return "MOCK_MJCF"
"""
    errors = run_linter(code)
    assert len(errors) == 0


def test_run_linter_syntax_error():
    code = """
def build(seed=0, scale=(1,1,1))
    return "MISSING COLON"
"""
    errors = run_linter(code)
    assert len(errors) > 0
    assert any("Ruff" in e for e in errors) or any("Pyrefly" in e for e in errors)


def test_run_linter_undefined_name():
    code = """
def build(seed=0, scale=(1,1,1)):
    x = undefined_variable
    return "MOCK"
"""
    errors = run_linter(code)
    assert len(errors) > 0
    # Pyrefly or Ruff should catch this
    assert any("undefined_variable" in e for e in errors)


# Note: we do NOT want unused imports. we want only hard, blocking errors.
# def test_run_linter_unused_import():
#     code = """
# import os
# def build(seed=0, scale=(1,1,1)):
#     return "MOCK"
# """
#     errors = run_linter(code)
#     assert len(errors) > 0
#     assert any("os" in e and "unused" in e.lower() for e in errors)


def test_run_linter_filtering_ruff():
    # F401: unused import (should be filtered)
    # F821: undefined name (should NOT be filtered)
    code = """
import os
def build(seed=0, scale=(1,1,1)):
    return undefined_func()
"""
    errors = run_linter(code)
    # Should only have 1 error (undefined name)
    assert len(errors) == 1
    assert "undefined_func" in errors[0]
    assert "unused" not in errors[0].lower()


def test_run_linter_max_errors():
    # Generate many errors
    code = "\n".join([f"x{i} = undefined_{i}" for i in range(20)])
    errors = run_linter(code)
    # Should be limited by MAX_ERRORS (default 10 in config if not overridden)
    assert len(errors) <= 10


@patch("src.generators.benchmark.linter.subprocess.run")
def test_run_linter_pyrefly_filtering(mock_run):
    # Mock Ruff to return no errors
    mock_ruff_res = MagicMock()
    mock_ruff_res.stdout = "[]"

    # Mock Pyrefly to return a warning and an error
    mock_pyrefly_res = MagicMock()
    mock_pyrefly_res.stdout = json.dumps(
        {
            "diagnostics": [
                {
                    "severity": "warning",
                    "name": "type-warning",
                    "description": "This is a warning",
                    "line": 1,
                },
                {
                    "severity": "error",
                    "name": "type-error",
                    "description": "This is an error",
                    "line": 2,
                },
            ]
        }
    )

    mock_run.side_effect = [mock_ruff_res, mock_pyrefly_res]

    # Re-run linter to use mocks
    errors = run_linter("some code")

    # Warning should be filtered if IGNORE_WARNINGS is True (default)
    assert len(errors) == 1
    assert "type-error" in errors[0]
    assert "warning" not in errors[0].lower()
