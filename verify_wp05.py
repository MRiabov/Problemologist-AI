import sys
import os
from pathlib import Path

# Add current directory to path so we can import src
sys.path.append(str(Path(__file__).parent))

from worker.runtime.executor import run_python_code, RuntimeConfig

code = """
import sys
from utils import simulate, validate_and_price
from build123d import Box
import os

box = Box(1, 1, 1)
v = validate_and_price(box)
print(f"VALIDATION: {v['valid']}, COST: {v['cost']}")

os.environ['CONTROLLER_URL'] = 'http://localhost:8000'
s = simulate(box)
print(f"SIMULATION SUCCESS: {s.success}")
print(s.summary)
"""

if __name__ == "__main__":
    # Use the same python executable that is running this script
    config = RuntimeConfig(python_executable=sys.executable)
    
    result = run_python_code(code, config=config)
    print("STDOUT:")
    print(result.stdout)
    print("STDERR:")
    print(result.stderr)
    print(f"EXIT CODE: {result.exit_code}")