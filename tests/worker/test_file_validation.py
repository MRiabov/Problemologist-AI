"""Tests for worker/utils/file_validation.py utilities."""

import pytest

from worker.utils.file_validation import (
    validate_objectives_yaml,
    validate_plan_md_structure,
    validate_review_frontmatter,
)


class TestValidateObjectivesYaml:
    """Tests for objectives.yaml validation."""

    @pytest.fixture
    def valid_yaml_content(self):
        return """
objectives:
  goal_zone:
    min: [0, 0, 0]
    max: [10, 10, 10]
  forbid_zones:
    - name: obstacle
      min: [5, 5, 0]
      max: [7, 7, 5]
  build_zone:
    min: [-20, -20, 0]
    max: [20, 20, 50]
simulation_bounds:
  min: [-50, -50, 0]
  max: [50, 50, 100]
moved_object:
  label: ball
  shape: sphere
  start_position: [0, 0, 30]
  runtime_jitter: [2, 2, 1]
constraints:
  max_unit_cost: 50.0
  max_weight: 1.2
"""

    def test_valid_content(self, valid_yaml_content):
        is_valid, result = validate_objectives_yaml(valid_yaml_content)
        assert is_valid is True
        assert result.moved_object.label == "ball"

    def test_empty_content(self):
        is_valid, errors = validate_objectives_yaml("")
        assert is_valid is False
        assert "Empty or invalid YAML content" in errors

    def test_missing_required_field(self):
        yaml_content = """
objectives:
  goal_zone:
    min: [0, 0, 0]
    max: [10, 10, 10]
  build_zone:
    min: [0, 0, 0]
    max: [10, 10, 10]
"""
        is_valid, errors = validate_objectives_yaml(yaml_content)
        assert is_valid is False
        assert any("simulation_bounds" in str(e) for e in errors)

    def test_invalid_yaml_syntax(self):
        is_valid, errors = validate_objectives_yaml("invalid: yaml: content:")
        assert is_valid is False


class TestValidateReviewFrontmatter:
    """Tests for review markdown frontmatter validation."""

    def test_valid_approved(self):
        content = """---
decision: approved
comments:
  - Great work
---

# Review

The design looks good.
"""
        is_valid, result = validate_review_frontmatter(content)
        assert is_valid is True
        assert result.decision == "approved"
        assert result.comments == ["Great work"]

    def test_valid_rejected(self):
        content = """---
decision: rejected
comments:
  - Geometry issue
  - Cost too high
---
"""
        is_valid, result = validate_review_frontmatter(content)
        assert is_valid is True
        assert result.decision == "rejected"
        assert len(result.comments) == 2

    def test_missing_frontmatter(self):
        content = "# Review\n\nNo frontmatter here."
        is_valid, errors = validate_review_frontmatter(content)
        assert is_valid is False
        assert any("Missing YAML frontmatter" in e for e in errors)

    def test_invalid_decision_value(self):
        content = """---
decision: maybe_approved
---
"""
        is_valid, errors = validate_review_frontmatter(content)
        assert is_valid is False

    def test_refusal_decision_without_context(self):
        """confirm_plan_refusal only valid if CAD agent actually refused."""
        content = """---
decision: confirm_plan_refusal
---
"""
        # Without cad_agent_refused=True, this should fail
        is_valid, errors = validate_review_frontmatter(content, cad_agent_refused=False)
        assert is_valid is False
        assert any("only valid when CAD agent refused" in e for e in errors)

    def test_refusal_decision_with_context(self):
        """confirm_plan_refusal valid when CAD agent actually refused."""
        content = """---
decision: confirm_plan_refusal
comments:
  - Plan geometry is indeed invalid
---
"""
        is_valid, result = validate_review_frontmatter(content, cad_agent_refused=True)
        assert is_valid is True
        assert result.decision == "confirm_plan_refusal"


class TestValidatePlanMdStructure:
    """Tests for plan.md structure validation."""

    def test_valid_benchmark_plan(self):
        content = """# Benchmark Plan

## Learning Objective
Test gravity manipulation

## Geometry
A funnel shape

## Objectives
Goal zone at bottom
"""
        is_valid, errors = validate_plan_md_structure(content, plan_type="benchmark")
        assert is_valid is True
        assert errors == []

    def test_missing_section_benchmark(self):
        content = """# Benchmark Plan

## Learning Objective
Test gravity
"""
        is_valid, errors = validate_plan_md_structure(content, plan_type="benchmark")
        assert is_valid is False
        assert any("Geometry" in e for e in errors)
        assert any("Objectives" in e for e in errors)

    def test_valid_engineering_plan(self):
        content = """# Engineering Plan

## 1. Solution Overview
A ramp design

## 2. Parts List
- Part A
- Part B

## 3. Assembly Strategy
1. Connect parts
2. Verify alignment

## 4. Cost & Weight Budget
- Part A: $1.00
- Part B: $2.00

## 5. Risk Assessment
- Low risk
"""
        is_valid, errors = validate_plan_md_structure(content, plan_type="engineering")
        assert is_valid is True

    def test_missing_section_engineering(self):
        content = """# Engineering Plan

## 1. Solution Overview
A design

## 2. Parts List
- Part A
"""
        is_valid, errors = validate_plan_md_structure(content, plan_type="engineering")
        assert is_valid is False
        assert any("Assembly Strategy" in e for e in errors)
