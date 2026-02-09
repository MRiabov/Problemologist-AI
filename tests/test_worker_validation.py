import os
from pathlib import Path

from build123d import Box, BuildPart, Compound

from worker.utils.validation import simulate, validate


def test_geometric_validation():
    # 1. Valid box
    with BuildPart() as p:
        Box(10, 10, 10)

    print(f"Validating single box: {validate(p.part)}")
    assert validate(p.part) == True

    # 2. Overlapping boxes
    with BuildPart() as p1:
        Box(10, 10, 10)
    with BuildPart() as p2:
        Box(10, 10, 10)

    comp = Compound(label="overlapping", children=[p1.part, p2.part])
    print(f"Validating overlapping boxes: {validate(comp)}")
    assert validate(comp) == False


def test_simulation():
    # Valid stable box
    with BuildPart() as p:
        Box(10, 10, 10)

    res = simulate(p.part)
    print(f"Simulation result: {res.success}, {res.summary}")
    assert res.success == True


def test_handover():
    from worker.utils.handover import submit_for_review

    with BuildPart() as p:
        Box(10, 10, 10)

    os.environ["SESSION_ID"] = "test_session"
    res = submit_for_review(p.part)
    assert res == True

    renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))
    manifest_path = renders_dir / "review_manifest.json"
    assert manifest_path.exists()

    with open(manifest_path) as f:
        import json

        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test_session"
        assert len(manifest["renders"]) == 24


if __name__ == "__main__":
    # Use local renders dir
    os.environ["RENDERS_DIR"] = "./renders"
    Path("./renders").mkdir(parents=True, exist_ok=True)
    test_geometric_validation()
    test_simulation()
    test_handover()
