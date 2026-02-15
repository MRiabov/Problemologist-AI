import pytest
from pydantic import ValidationError
from shared.models.steerability import (
    GeometricSelection,
    SelectionLevel,
    CodeReference,
    SteerablePrompt,
)


def test_geometric_selection_valid():
    selection = GeometricSelection(
        level=SelectionLevel.FACE,
        target_id="part1.face[0]",
        center=(1.0, 2.0, 3.0),
        normal=(0.0, 0.0, 1.0),
    )
    assert selection.level == SelectionLevel.FACE
    assert selection.center == (1.0, 2.0, 3.0)
    assert selection.normal == (0.0, 0.0, 1.0)
    assert selection.direction is None


def test_geometric_selection_invalid_vector():
    with pytest.raises(ValidationError):
        # Center must be 3 elements
        GeometricSelection(
            level=SelectionLevel.PART, target_id="part1", center=(1.0, 2.0)
        )

    with pytest.raises(ValidationError):
        # Center must be 3 elements
        GeometricSelection(
            level=SelectionLevel.PART, target_id="part1", center=(1.0, 2.0, 3.0, 4.0)
        )


def test_code_reference_valid():
    ref = CodeReference(file_path="script.py", start_line=10, end_line=15)
    assert ref.file_path == "script.py"
    assert ref.start_line == 10
    assert ref.end_line == 15


def test_steerable_prompt_valid():
    prompt = SteerablePrompt(
        text="Make this face smoother",
        selections=[
            GeometricSelection(
                level=SelectionLevel.FACE, target_id="f1", center=(0, 0, 0)
            )
        ],
        code_references=[CodeReference(file_path="main.py", start_line=1, end_line=2)],
        mentions=["@smoother"],
    )
    assert len(prompt.selections) == 1
    assert prompt.mentions == ["@smoother"]


def test_steerable_prompt_defaults():
    prompt = SteerablePrompt(text="Simple prompt")
    assert prompt.selections == []
    assert prompt.code_references == []
    assert prompt.mentions == []
