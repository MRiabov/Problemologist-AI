import sys
import os


def test_import(module_name):
    print(f"Testing import of {module_name}...")
    try:
        __import__(module_name)
        print(f"Successfully imported {module_name}")
    except Exception as e:
        print(f"Failed to import {module_name}: {e}")


if __name__ == "__main__":
    # Test common suspects
    test_import("OpenGL")
    test_import("genesis")
    test_import("mujoco")
    test_import("trimesh")
    test_import("matplotlib.pyplot")
    test_import("worker.api.routes")
