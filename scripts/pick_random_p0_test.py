import random
import subprocess
import sys

def get_p0_tests():
    try:
        result = subprocess.run(
            ["uv", "run", "pytest", "-m", "integration_p0", "--collect-only", "-q"],
            capture_output=True,
            text=True,
            check=True
        )
        tests = [line.strip() for line in result.stdout.splitlines() if "::" in line]
        return tests
    except subprocess.CalledProcessError as e:
        print(f"Error collecting tests: {e.stderr}", file=sys.stderr)
        return []

def main():
    tests = get_p0_tests()
    if not tests:
        print("No P0 tests found.")
        sys.exit(1)

    selected_test = random.choice(tests)
    print(selected_test)

if __name__ == "__main__":
    main()
