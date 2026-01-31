---
trigger: always_on
glob:
description: When running python files or running spec-kitty
---
There is a .venv at project root, so if you have trouble installing packages or running `spec-kitty`, do `source .venv/bin/activate` (check if you are in the correct folder) and run with it.

You could also use `uv run spec-kitty ...` for running spec-kitty, but the `venv` approach is preferable.

