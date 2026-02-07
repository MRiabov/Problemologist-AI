import sys
import asyncio
from typing import Any
from build123d import Part, Box
from shared.workbenches.print_3d import Print3DWorkbench
from shared.workbenches.models import ManufacturingConfig
from shared.cots.indexer import Indexer
from controller.agent.nodes.architect import architect_node
from controller.workflows.simulation import compile_mjcf_activity
from beartype.roar import BeartypeCallHintParamViolation


def test_workbench_validation():
    print("Testing Workbench type checking...")
    wb = Print3DWorkbench()

    # Positive case
    part = Part(Box(10, 10, 10))
    try:
        wb.validate(part)
        print("✅ Workbench.validate(Part) passed.")
    except Exception as e:
        print(f"❌ Workbench.validate(Part) failed: {e}")
        sys.exit(1)

    # Negative case
    try:
        wb.validate("not a part")  # type: ignore
        print("❌ Workbench.validate(str) check FAILED (should have raised exception).")
        sys.exit(1)
    except BeartypeCallHintParamViolation:
        print(
            "✅ Workbench.validate(str) correctly raised BeartypeCallHintParamViolation."
        )
    except Exception as e:
        print(
            f"❌ Workbench.validate(str) raised unexpected exception: {type(e).__name__}: {e}"
        )
        sys.exit(1)


def test_indexer_validation():
    print("\nTesting Indexer type checking...")
    indexer = Indexer("test.db")

    # Negative case for helper method
    try:
        # generate_recipe expects (Type, dict), passing (str, dict)
        indexer.generate_recipe("not a type", {})  # type: ignore
        print("❌ Indexer.generate_recipe(str, dict) check FAILED.")
        sys.exit(1)
    except BeartypeCallHintParamViolation:
        print(
            "✅ Indexer.generate_recipe(str, dict) correctly raised BeartypeCallHintParamViolation."
        )
    except Exception as e:
        print(
            f"❌ Indexer.generate_recipe(str, dict) raised unexpected exception: {type(e).__name__}: {e}"
        )
        sys.exit(1)


def test_agent_node_validation():
    print("\nTesting Agent Node type checking...")
    # architect_node expects (AgentState), passing (dict)
    try:
        architect_node({"invalid": "state"})  # type: ignore
        print("❌ architect_node(dict) check FAILED.")
        sys.exit(1)
    except BeartypeCallHintParamViolation:
        print(
            "✅ architect_node(dict) correctly raised BeartypeCallHintParamViolation."
        )
    except Exception as e:
        print(
            f"❌ architect_node(dict) raised unexpected exception: {type(e).__name__}: {e}"
        )
        sys.exit(1)


async def test_controller_activity_validation():
    print("\nTesting Controller Activity type checking...")
    # compile_mjcf_activity expects (str), passing (int)
    try:
        await compile_mjcf_activity(123)  # type: ignore
        print("❌ compile_mjcf_activity(int) check FAILED.")
        sys.exit(1)
    except BeartypeCallHintParamViolation:
        print(
            "✅ compile_mjcf_activity(int) correctly raised BeartypeCallHintParamViolation."
        )
    except Exception as e:
        print(
            f"❌ compile_mjcf_activity(int) raised unexpected exception: {type(e).__name__}: {e}"
        )
        sys.exit(1)


if __name__ == "__main__":
    test_workbench_validation()
    test_indexer_validation()
    test_agent_node_validation()
    asyncio.run(test_controller_activity_validation())
    print("\n🎉 All Beartype verification tests passed!")
