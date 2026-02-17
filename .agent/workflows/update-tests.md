---
description: Update tests after a refactor or logic change.
---

I have performed a major refactor. The code in the /src directory is the intent and is a source of truth. Notably, the real source of truth is in @specs/desired_architecture.md and other specs/, but my code fills the gaps that those document don't.

The tests in /tests are currently failing because they are outdated and stale.

Your Goal: Update the unit tests to match the new behavior of the code.

Constraints:

You have READ-ONLY access to production code (e.g., `controller/`, `shared/`, `frontend/`, `). You must NOT modify these files under any circumstances.

You only have WRITE access to test files (e.g., *.test.ts, test_*.py).

If a test expects Output A but the code now returns Output B, change the test to expect Output B. Do not change the code to return Output A."

Workflow:
We have a signficant test coverage in Python. Do not run all tests at once, instead, use `uv run pytest --maxfail 10` and fix the first ones. Update the tests and do not introduce new code into the prod code.
If something appears like a bug in *my* code, remember it notify me in the end of the session, I'll fix; however until then proceed with moving forward with fixing other tests.
Focus on solving hard errors e.g. ImportErrors and ModuleNotFound errors before you focus on failing assertions.
