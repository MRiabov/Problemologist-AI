from build123d import Box, Compound, BuildPart
from src.worker.utils.validation import validate, simulate
import os

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
    from src.worker.utils.handover import submit_for_review
    with BuildPart() as p:
        Box(10, 10, 10)
    
    os.environ["SESSION_ID"] = "test_session"
    res = submit_for_review(p.part)
    assert res == True
    
    renders_dir = os.getenv("RENDERS_DIR", "./renders")
    manifest_path = os.path.join(renders_dir, "review_manifest.json")
    assert os.path.exists(manifest_path)
    
    with open(manifest_path, "r") as f:
        import json
        manifest = json.load(f)
        assert manifest["status"] == "ready_for_review"
        assert manifest["session_id"] == "test_session"
        assert len(manifest["renders"]) == 24

if __name__ == "__main__":
    # Use local renders dir
    os.environ["RENDERS_DIR"] = "./renders"
    os.makedirs("./renders", exist_ok=True)
    test_geometric_validation()
    test_simulation()