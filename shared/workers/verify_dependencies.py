"""Test script to verify CAD dependencies are available.

This verifies that build123d and mujoco can be imported
in the worker environment.
"""

import sys


def verify_build123d() -> bool:
    """Verify build123d can be imported."""
    try:
        import build123d

        print(f"✓ build123d version: {build123d.__version__}")
        return True
    except ImportError as e:
        print(f"✗ Failed to import build123d: {e}")
        return False


def verify_mujoco() -> bool:
    """Verify mujoco can be imported."""
    try:
        import mujoco

        print(f"✓ mujoco version: {mujoco.__version__}")
        return True
    except ImportError as e:
        print(f"✗ Failed to import mujoco: {e}")
        return False


def main() -> int:
    """Run all dependency verification checks."""
    print("Verifying CAD dependencies...\n")

    results = [
        verify_build123d(),
        verify_mujoco(),
    ]

    print()
    if all(results):
        print("All dependencies verified successfully!")
        return 0
    print("Some dependencies failed verification.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
