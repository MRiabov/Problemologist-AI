"""Compatibility CLI entrypoint for skill overlay promotion.

Implementation lives in evals.logic.skill_promotion.
"""

import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    from evals.logic.skill_promotion import main as promote_main

    raise SystemExit(promote_main())


if __name__ == "__main__":
    main()
