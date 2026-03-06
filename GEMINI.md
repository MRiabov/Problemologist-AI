# Mandatory rules for agents

## Important context and specs

To save me refactors and to speed up debugging, please read @specs/desired_architecture.md on any of my requests.

## Rules

In this repo, the convention is to not use unit tests because they are flaky, but to use only real integration tests. They are faster to debug against. So, when implementing new logic, verify it against integration tests. The integration test list and spec can be found at @specs/integration-tests.md

---

This is all for the critical rules, but please follow the two context rules, they reference two most important documents in the architecture.
