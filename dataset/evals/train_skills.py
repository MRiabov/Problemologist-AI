"""Compatibility CLI entrypoint for retained-bundle skill training.

Implementation lives in evals.logic.skill_training.
"""

import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    from evals.logic.skill_training import run_cli

    run_cli()


if __name__ == "__main__":
    main()
