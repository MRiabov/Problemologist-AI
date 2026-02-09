"""Tests for shared/models/schemas.py Pydantic models."""

import pytest
from pydantic import ValidationError

from shared.models.schemas import (
    BoundingBox,
    Constraints,
    MovedObject,
    MovingPart,
    ObjectivesSection,
    ObjectivesYaml,
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
        fm = ReviewFrontmatter(decision="approved", comments=["Looks good"])
        assert fm.decision == "approved"
        assert fm.comments == ["Looks good"]

    def test_rejected_decision(self):
        fm = ReviewFrontmatter(decision="rejected", comments=["Fix geometry"])
        assert fm.decision == "rejected"

    def test_confirm_plan_refusal(self):
        fm = ReviewFrontmatter(decision="confirm_plan_refusal")
        assert fm.decision == "confirm_plan_refusal"

    def test_reject_plan_refusal(self):
        fm = ReviewFrontmatter(decision="reject_plan_refusal")
        assert fm.decision == "reject_plan_refusal"

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
            name="feeder",
            type="motor",
            position=(0, 0, 0),
            dof="rotate_z",
            description="Rotation motor",
        )
        assert part.type == "motor"
        assert part.dof == "rotate_z"

    def test_passive_type(self):
        part = MovingPart(
            name="slider",
            type="passive",
            position=[5, 5, 0],
            dof="slide_y",
            description="Sliding element",
        )
        assert part.type == "passive"
        assert part.position == (5, 5, 0)

    def test_invalid_dof(self):
        with pytest.raises(ValidationError):
            MovingPart(
                name="test",
                type="motor",
                position=(0, 0, 0),
                dof="spin",  # Invalid
                description="test",
            )


class TestObjectivesYaml:
    """Tests for full ObjectivesYaml schema."""

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
            "constraints": {"max_unit_cost": 50.0, "max_weight": 1.2},
        }

    def test_valid_full_schema(self, valid_objectives_data):
        obj = ObjectivesYaml(**valid_objectives_data)
        assert obj.objectives.goal_zone.min == (0, 0, 0)
        assert obj.moved_object.label == "ball"
        assert obj.constraints.max_unit_cost == 50.0

    def test_missing_constraints(self, valid_objectives_data):
        del valid_objectives_data["constraints"]
        with pytest.raises(ValidationError):
            ObjectivesYaml(**valid_objectives_data)

    def test_missing_objectives_section(self, valid_objectives_data):
        del valid_objectives_data["objectives"]
        with pytest.raises(ValidationError):
            ObjectivesYaml(**valid_objectives_data)

    def test_with_moving_parts(self, valid_objectives_data):
        valid_objectives_data["moving_parts"] = [
            {
                "name": "motor1",
                "type": "motor",
                "position": [0, 0, 0],
                "dof": "rotate_z",
                "description": "Main motor",
            }
        ]
        obj = ObjectivesYaml(**valid_objectives_data)
        assert len(obj.moving_parts) == 1
        assert obj.moving_parts[0].name == "motor1"
