# Test Framework

This repository is backend-first. Most verification happens through
`./scripts/run_integration_tests.sh` and the HTTP/system integration suites
under `tests/integration/`. Browser automation is a narrow optional layer for
the small set of live UI flows that need a real browser, implemented with
`pytest-playwright`.

Integration test additions, removals, renames, and scope changes must keep
their `INT-xxx` mapping in `specs/integration-test-list.md`.

Optional browser tests live under:

- `tests/integration/frontend/`
- `tests/e2e/`
- shared support code in `tests/support/`

## Setup

The canonical integration entrypoint is:

```bash
./scripts/run_integration_tests.sh
```

That script starts the compose stack, runs migrations, starts backend services,
and then runs pytest with the repo's integration filters. It only builds and
serves the frontend when the browser slice is selected.

By default, a full run is split into ordered marker buckets so release-blocking
coverage lands first:

1. `integration_p0`
2. `integration_p1`
3. `integration_agent`
4. `integration_frontend`
5. `integration_p2`

The bucket expressions are disjoint, so a test runs once. Overlaps such as
`integration_agent` + `integration_p1` stay in the later category bucket rather
than being duplicated.
Set `INTEGRATION_ORDERED_MARKER_SPLITS=0` if you want the old single-pass
pytest behavior.

Integration session IDs use the `INT-{number}-{uuid8}` pattern. Keep that
shape in any test-authored artifacts or assertions that need to correlate a
run with logs or persisted records.

For backend-first integration work, use the marker-driven runner directly:

```bash
./scripts/run_integration_tests.sh -m integration_p0
```

For the optional browser slice, use:

```bash
./scripts/run_integration_tests.sh -m integration_frontend
```

The same optional slice is available through the convenience target:

```bash
make test-frontend
```

For just the `tests/e2e/` browser-smoke folder, use:

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
- Use `pytest.mark.xdist_group(...)` only for tests that truly cannot run
  concurrently; do not infer serial lanes from module contents or helper names.
- Prefer shared helpers and page objects over inline polling loops.
- Keep assertions on observable UI state and backend responses.
- Avoid arbitrary sleeps. Poll for a concrete DOM or API condition instead.
- Use the shared browser helpers for navigation and status polling.
- Keep the test traffic at the browser, API, or storage boundary. Do not
  invoke controller or worker internals directly when the test is meant to be
  integration coverage.

## Notes

- Browser console/page errors are captured by `tests/conftest.py`.
- The repo does not use a separate Node `playwright test` runner; Playwright is
  integrated through `pytest-playwright`.
- The browser tests are designed to run against the real compose stack, not
  controller or worker internals.
