from src.generators.benchmark.agent import coder_node, planner_node, validator_node
from src.generators.benchmark.renderer import render_scenario


def test_imports():
    print("Checking imports...")
    assert planner_node is not None
    assert coder_node is not None
    assert validator_node is not None
    assert render_scenario is not None
    print("Imports OK!")


if __name__ == "__main__":
    try:
        test_imports()
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback

        traceback.print_exc()
        exit(1)
