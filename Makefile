.PHONY: test test-integration test-frontend test-e2e

test:
	uv run pytest

test-integration:
	./scripts/run_integration_tests.sh -m integration

test-frontend:
	./scripts/run_integration_tests.sh -m integration_frontend

test-e2e:
	./scripts/run_integration_tests.sh tests/e2e
