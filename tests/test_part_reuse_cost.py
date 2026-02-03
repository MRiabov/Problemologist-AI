import sys
from pathlib import Path
import build123d as bd

# Add src to path
sys.path.append(str(Path(__file__).resolve().parent.parent))

from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench


def test_cnc_reuse_discount():
    workbench = CNCWorkbench()
    part = bd.Box(10, 10, 10)

    # 1. First part cost
    context = {}
    res1 = workbench.calculate_cost(part, quantity=1, context=context)
    cost1 = res1["total_cost"]

    # 2. Identical part cost (same center/volume)
    res2 = workbench.calculate_cost(part, quantity=1, context=context)
    cost2 = res2["total_cost"]

    print(f"CNC: Cost 1: {cost1:.2f}, Cost 2: {cost2:.2f}")
    # Setup cost is material_cfg["machine_hourly_rate"] which is 80.0 in config for alu
    # Cost 2 should be cost1 - (80.0 * 0.5) = cost1 - 40.0
    assert cost2 < cost1
    # Check if saving is exactly 40.0
    diff = cost1 - cost2
    print(f"CNC Saving: {diff:.2f}")
    assert abs(diff - 40.0) < 0.01


def test_im_reuse_discount():
    workbench = InjectionMoldingWorkbench()
    part = bd.Box(10, 10, 10)

    # 1. First part cost
    context = {}
    res1 = workbench.calculate_cost(part, quantity=100, context=context)
    cost1 = res1["total_cost"]

    # 2. Identical part cost
    res2 = workbench.calculate_cost(part, quantity=100, context=context)
    cost2 = res2["total_cost"]

    print(f"IM: Cost 1: {cost1:.2f}, Cost 2: {cost2:.2f}")
    # IM Tooling cost is ~2000. Reusing it gives 90% discount (saves 1800)
    assert cost2 < cost1
    assert (cost1 - cost2) > 1000  # Significant saving


if __name__ == "__main__":
    import pytest

    # Run tests manually if not using pytest runner
    test_cnc_reuse_discount()
    test_im_reuse_discount()
    print("Tests passed!")
