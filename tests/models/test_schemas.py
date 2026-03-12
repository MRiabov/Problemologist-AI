"""Tests for shared/models/schemas.py Pydantic models."""

import pytest
from pydantic import ValidationError

from shared.models.schemas import (
    BenchmarkDefinition,
    BoundingBox,
    MovingPart,
    ReviewFrontmatter,
)


class TestBoundingBox:
    """Tests for BoundingBox model."""

    def test_valid_bounding_box(self):
        bbox = BoundingBox(min=(0, 0, 0), max=(10, 10, 10))
        assert bbox.min == (0, 0, 0)
        assert bbox.max == (10, 10, 10)

    def test_list_to_tuple_coercion(self):
        """YAML typically loads as lists, should coerce to tuples."""
        bbox = BoundingBox(min=[0, 0, 0], max=[10, 10, 10])
        assert bbox.min == (0, 0, 0)
        assert bbox.max == (10, 10, 10)

    def test_invalid_coordinates_count(self):
        with pytest.raises(ValidationError):
            BoundingBox(min=(0, 0), max=(10, 10, 10))  # Only 2 coords


class TestReviewFrontmatter:
    """Tests for ReviewFrontmatter model."""

    def test_approved_decision(self):
        fm = ReviewFrontmatter(decision="APPROVED", comments=["Looks good"])
        assert fm.decision == "APPROVED"
        assert fm.comments == ["Looks good"]

    def test_rejected_decision(self):
        fm = ReviewFrontmatter(decision="REJECTED", comments=["Fix geometry"])
        assert fm.decision == "REJECTED"

    def test_confirm_plan_refusal(self):
        fm = ReviewFrontmatter(decision="CONFIRM_PLAN_REFUSAL")
        assert fm.decision == "CONFIRM_PLAN_REFUSAL"

    def test_reject_plan_refusal(self):
        fm = ReviewFrontmatter(decision="REJECT_PLAN_REFUSAL")
        assert fm.decision == "REJECT_PLAN_REFUSAL"

    def test_invalid_decision(self):
        with pytest.raises(ValidationError):
            ReviewFrontmatter(decision="maybe")

    def test_empty_comments_default(self):
        fm = ReviewFrontmatter(decision="approved")
        assert fm.comments == []


class TestMovingPart:
    """Tests for MovingPart model."""

    def test_motor_type(self):
        part = MovingPart(
            part_name="feeder",
            type="MOTOR",
            dofs=["rotate_z"],
        )
        assert part.type == "MOTOR"
        assert "rotate_z" in part.dofs

    def test_passive_type(self):
        part = MovingPart(
            part_name="slider",
            type="PASSIVE",
            dofs=["slide_y"],
        )
        assert part.type == "PASSIVE"
        assert "slide_y" in part.dofs


class TestBenchmarkDefinition:
    """Tests for full BenchmarkDefinition schema."""

    @pytest.fixture
    def valid_objectives_data(self):
        return {
            "objectives": {
                "goal_zone": {"min": [0, 0, 0], "max": [10, 10, 10]},
                "forbid_zones": [
                    {"name": "obstacle", "min": [5, 5, 0], "max": [7, 7, 5]}
                ],
                "build_zone": {"min": [-20, -20, 0], "max": [20, 20, 50]},
            },
            "simulation_bounds": {"min": [-50, -50, 0], "max": [50, 50, 100]},
            "moved_object": {
                "label": "ball",
                "shape": "sphere",
                "start_position": [0, 0, 30],
                "runtime_jitter": [2, 2, 1],
            },
            "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1.2},
        }

    def test_valid_full_schema(self, valid_objectives_data):
        obj = BenchmarkDefinition(**valid_objectives_data)
        assert obj.objectives.goal_zone.min == (0, 0, 0)
        assert obj.moved_object.label == "ball"
        assert obj.constraints.max_unit_cost == 50.0

    def test_missing_constraints(self, valid_objectives_data):
        del valid_objectives_data["constraints"]
        with pytest.raises(ValidationError):
            BenchmarkDefinition(**valid_objectives_data)

    def test_missing_objectives_section(self, valid_objectives_data):
        del valid_objectives_data["objectives"]
        with pytest.raises(ValidationError):
            BenchmarkDefinition(**valid_objectives_data)
