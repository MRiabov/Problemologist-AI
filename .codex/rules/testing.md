---
trigger: always_on
---

Mock tests are sometimes good, but in no more than 20-40% of cases. Instead of writing mock tests, better make tests that test the entire application/module logic. This makes for robust and fail-less pipelines, whereas mock tests can fail in catching bugs.

For integration work, use `specs/integration-test-rules.md` for the rules/workflow guidance and `specs/integration-test-list.md` for the canonical `INT-xxx` / `INT-NEG-###` catalog.
