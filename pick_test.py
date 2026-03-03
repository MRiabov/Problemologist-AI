import random
import glob
import os

def main():
    # Find all frontend integration tests
    test_patterns = [
        "tests/integration/frontend/*.py",
        "tests/integration/frontend/p0/*.py",
        "tests/integration/frontend/p1/*.py",
    ]

    all_tests = []
    for pattern in test_patterns:
        all_tests.extend(glob.glob(pattern))

    # Filter out __init__.py
    all_tests = [t for t in all_tests if not t.endswith("__init__.py")]

    if not all_tests:
        print("No frontend integration tests found.")
        return

    print(random.choice(all_tests))

if __name__ == "__main__":
    main()
