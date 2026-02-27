import subprocess
import random
import re

def get_p0_tests():
    result = subprocess.run(
        ["uv", "run", "pytest", "--collect-only", "-m", "integration_p0", "tests/integration"],
        capture_output=True,
        text=True
    )
    # Extract test names. They look like <Coroutine test_int_001_compose_boot_health_contract>
    # or just the name if it's not a coroutine.
    # Actually, the output has structure.
    tests = []
    for line in result.stdout.splitlines():
        match = re.search(r'<(?:Coroutine|Function|Method) (.*)>', line)
        if match:
            tests.append(match.group(1))
    return tests

if __name__ == "__main__":
    tests = get_p0_tests()
    if not tests:
        print("No tests found")
    else:
        print(random.choice(tests))
