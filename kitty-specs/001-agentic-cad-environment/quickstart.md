# Quickstart: Agentic CAD Environment

## Installation

```bash
# From project root
pip install -r requirements.txt
```

## Usage

```python
from src.environment.core import CADEnvironment

# 1. Initialize
env = CADEnvironment(db_path="history.db")
obs = env.reset(problem_id="move_block_001")

print(f"Task: {obs.task_description}")

# 2. Agent Loop
# (Agent logic would go here)

# Example: Write a script
action = {
    "tool_name": "write_script",
    "content": "from build123d import *; b = Box(10,10,10)"
}
obs, reward, done, info = env.step(**action)

print(f"Output: {obs.console_output}")

# 3. Submit
action = {"tool_name": "submit_design"}
obs, reward, done, info = env.step(**action)

if done:
    print(f"Final Metrics: {info['metrics']}")
```

## Running Tests

```bash
pytest tests/environment
```
