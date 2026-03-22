# Test Framework

This repository uses `pytest` as the primary test runner, with `pytest-playwright`
providing the browser automation layer for the frontend and e2e suite.

The browser tests live under:

- `tests/e2e/`
- `tests/integration/frontend/`
- shared support code in `tests/support/`

## Setup

The canonical integration entrypoint is:

```bash
./scripts/run_integration_tests.sh
```

That script starts the compose stack, prepares the frontend build, and then runs
pytest with the repo's integration filters.

For browser-only coverage, use:

```bash
./scripts/run_integration_tests.sh -m integration_frontend
```

The same selection is available through the convenience target:

```bash
make test-frontend
```

For just the sample e2e folder, use:

```bash
make test-e2e
```

## Shared Support

Browser tests should import shared helpers from `tests/support/` instead of
duplicating page polling, navigation, or artifact-reading logic.

Current shared pieces:

- `tests/support/fixtures/browser.py`
- `tests/support/helpers/playwright.py`
- `tests/support/page_objects/benchmark_workspace.py`
- `tests/support/fixtures/factories/prompts.py`

`tests/conftest.py` loads the browser fixture plugin so these helpers are
available to the entire test tree.

## Fixtures and Environment

Useful env vars:

- `BASE_URL`: frontend base URL used by the browser fixtures
- `FRONTEND_URL`: alias for the frontend base URL
- `API_URL`: controller base URL
- `VITE_API_URL`: frontend API base URL for the Vite app
- `PLAYWRIGHT_ARTIFACTS_DIR`: local artifact output path

The default values are documented in `.env.example`.

## Best Practices

- Use `pytest.mark.integration_frontend` for browser tests that require the
  frontend stack.
- Prefer shared helpers and page objects over inline polling loops.
- Keep assertions on observable UI state and backend responses.
- Avoid arbitrary sleeps. Poll for a concrete DOM or API condition instead.
- Use the shared browser helpers for navigation and status polling.

## Notes

- Browser console/page errors are captured by `tests/conftest.py`.
- The repo does not use a separate Node `playwright test` runner; Playwright is
  integrated through `pytest-playwright`.
- The browser tests are designed to run against the real compose stack, not
  mocked controller or worker internals.
