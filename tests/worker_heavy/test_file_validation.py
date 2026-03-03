"""Tests for worker/utils/file_validation.py utilities."""

import pytest
import yaml

from worker_heavy.utils.file_validation import (
    validate_assembly_definition_yaml,
    validate_objectives_yaml,
    validate_plan_md_structure,
    validate_review_frontmatter,
)


class TestValidateObjectivesYaml:
    """Tests for objectives.yaml validation (INT-008)."""

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
  max_weight_g: 1200.0
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
        assert any("YAML parse error" in str(e) for e in errors)

    def test_template_placeholders(self):
        content = "objectives:\n  goal_zone:\n    min: [x_min, 0, 0]"
        is_valid, errors = validate_objectives_yaml(content)
        assert is_valid is False
        assert any("template placeholders" in str(e) for e in errors)

    def test_mujoco_fluid_rejection(self):
        content = """
objectives:
  goal_zone: {min: [0,0,0], max: [1,1,1]}
  forbid_zones: []
  build_zone: {min: [0,0,0], max: [1,1,1]}
simulation_bounds: {min: [0,0,0], max: [1,1,1]}
moved_object: {label: x, shape: sphere, start_position: [0,0,0], runtime_jitter: [0,0,0]}
constraints: {max_unit_cost: 1, max_weight_g: 1}
physics:
  backend: MUJOCO
fluids:
  - fluid_id: water
    initial_volume: {type: box, center: [0,0,0], size: [1,1,1]}
"""
        is_valid, errors = validate_objectives_yaml(content)
        assert is_valid is False
        assert "MuJoCo backend does not support fluids" in str(errors)


class TestValidatePlanMdStructure:
    """Tests for plan.md structure validation (INT-006)."""

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
        # validate_plan_md (from markdown_validator) returns list of missing sections
        assert any("Assembly Strategy" in e for e in errors)
        assert any("Cost & Weight Budget" in e for e in errors)
        assert any("Risk Assessment" in e for e in errors)


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
        assert result.decision.value == "APPROVED"
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
        assert result.decision.value == "REJECTED"
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
        # Without cad_agent_refused=False, this should fail
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
        assert result.decision.value == "CONFIRM_PLAN_REFUSAL"


class TestValidateTodoMd:
    """Tests for todo.md validation (INT-007)."""

    def test_invalid_checkbox(self):
        from shared.workers.markdown_validator import validate_todo_md

        content = "- [?] Invalid"
        res = validate_todo_md(content)
        assert not res.is_valid
        assert "invalid checkbox" in res.violations[0].lower()

    def test_require_completion(self):
        from shared.workers.markdown_validator import validate_todo_md

        content = "- [ ] Unfinished"
        res = validate_todo_md(content, require_completion=True)
        assert not res.is_valid
        assert "must be completed or skipped" in res.violations[0].lower()

    def test_valid_completion(self):
        from shared.workers.markdown_validator import validate_todo_md

        content = "- [x] Done\n- [-] Skipped"
        res = validate_todo_md(content, require_completion=True)
        assert res.is_valid


class TestValidateAssemblyDefinitionYaml:
    """Tests for assembly_definition.yaml validation (INT-009, INT-010, INT-011)."""

    @pytest.fixture
    def valid_asm_content(self):
        return """
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_g: 1000.0
  planner_target_max_unit_cost_usd: 80.0
  planner_target_max_weight_g: 800.0
totals:
  estimated_unit_cost_usd: 50.0
  estimated_weight_g: 500.0
  estimate_confidence: high
"""

    def test_valid_asm(self, valid_asm_content):
        is_valid, result = validate_assembly_definition_yaml(valid_asm_content)
        assert is_valid is True

    def test_planner_cap_exceeds_benchmark(self):
        content = """
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_g: 1000.0
  planner_target_max_unit_cost_usd: 120.0
  planner_target_max_weight_g: 800.0
totals:
  estimated_unit_cost_usd: 50.0
  estimated_weight_g: 500.0
  estimate_confidence: high
"""
        is_valid, errors = validate_assembly_definition_yaml(content)
        assert is_valid is False
        assert any("Planner target cost (120.0) must be less than or equal" in e for e in errors)

    def test_estimated_exceeds_planner_cap(self):
        content = """
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_g: 1000.0
  planner_target_max_unit_cost_usd: 80.0
  planner_target_max_weight_g: 800.0
totals:
  estimated_unit_cost_usd: 90.0
  estimated_weight_g: 500.0
  estimate_confidence: high
"""
        is_valid, errors = validate_assembly_definition_yaml(content)
        assert is_valid is False
        assert any("Estimated unit cost ($90.0) exceeds target limit ($80.0)" in e for e in errors)


class TestValidatePlanRefusal:
    """Tests for plan_refusal.md validation (INT-072)."""

    def test_valid_refusal(self):
        from worker_heavy.utils.file_validation import validate_plan_refusal

        content = """---
role: engineering_mechanical_coder
reasons:
  - PHYSICALLY_IMPOSSIBLE
---
Evidence goes here.
"""
        is_valid, result = validate_plan_refusal(content)
        assert is_valid is True

    def test_invalid_role_reasons(self):
        from worker_heavy.utils.file_validation import validate_plan_refusal

        content = """---
role: engineering_electrical_coder
reasons:
  - PHYSICALLY_IMPOSSIBLE
---
Evidence.
"""
        is_valid, errors = validate_plan_refusal(content)
        assert is_valid is False
        assert any("Invalid reasons for engineering_electrical_coder" in e for e in errors)

    def test_missing_evidence(self):
        from worker_heavy.utils.file_validation import validate_plan_refusal

        content = """---
role: engineering_mechanical_coder
reasons:
  - PHYSICALLY_IMPOSSIBLE
---
"""
        is_valid, errors = validate_plan_refusal(content)
        assert is_valid is False
        assert any("must include evidence in the body" in e for e in errors)
