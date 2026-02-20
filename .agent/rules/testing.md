---
trigger: always_on
---

Mock tests are sometimes good, but in no more than 20-40% of cases. Instead of writing mock tests, better make tests that test the entire application/module logic. This makes for robust and fail-less pipelines, whereas mock tests can fail in catching bugs.

Integration tests, which adhere to strict black-box rules (no mocks, imports from file, anything except mocking LLMs and external API calls) as described in @specs/integration-tests.md, are to be run with `./scripts/run-integration-tests.sh` to make it initiated simpler. There are a few useful flags on the script, like "integration_p0" which will run p0/p1/p2 tests (as per the @specs/integration-tests.md); and a particular script can be run via `./scripts/run-integration-tests.sh tests/integration/architecture_p0/test_architecture_p0.py`
Note that in integration tests, because of some persistence helper logic, we specify all session_ids as `INT-{test-number}-uuid[8:]`.
