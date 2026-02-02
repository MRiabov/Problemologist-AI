import pytest
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


def test_run_linter_unused_import():
    code = """
import os
def build(seed=0, scale=(1,1,1)):
    return "MOCK"
"""
    errors = run_linter(code)
    assert len(errors) > 0
    assert any("os" in e and "unused" in e.lower() for e in errors)
