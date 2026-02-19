import os
import random

tests_dir = "tests/integration/architecture_p0"
tests = [f for f in os.listdir(tests_dir) if f.startswith("test_") and f.endswith(".py")]
random_test = random.choice(tests)
print(f"{tests_dir}/{random_test}")
