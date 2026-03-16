"""CLI wrapper for eval runs.

All orchestration logic lives in controller.evals.runner.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controller.evals.runner import run_cli

if __name__ == "__main__":
    run_cli()
