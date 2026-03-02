import subprocess
import random
import sys

def main():
    try:
        # Dynamically discover p0 integration tests
        cmd = ["uv", "run", "pytest", "--collect-only", "-m", "integration_p0", "tests/integration", "-q"]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)

        # Parse the output to get test names
        tests = [line.strip() for line in result.stdout.split("\n") if line.strip() and not line.startswith("=") and "::" in line]

        if not tests:
            print("No integration_p0 tests found.")
            sys.exit(1)

        print(random.choice(tests))
    except Exception as e:
        print(f"Error picking random test: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
