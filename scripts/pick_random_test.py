import glob
import random

def pick_test():
    # Discover all integration tests in tests/integration/frontend
    tests = glob.glob("tests/integration/frontend/**/*.py", recursive=True)
    # Filter out __init__.py files
    tests = [t for t in tests if not t.endswith("__init__.py")]

    if not tests:
        print("No frontend integration tests found.")
        return

    print(random.choice(tests))

if __name__ == "__main__":
    pick_test()
