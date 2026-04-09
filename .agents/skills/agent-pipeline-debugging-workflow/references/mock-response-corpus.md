# Mock-Response Corpus Reference

## Corpus shape

- Keep one scenario file per test id under `tests/integration/mock_responses/`.
- Use strict filenames such as `INT-###.yaml`; follow the catalog for any negative IDs that are added.
- The loader fails closed on invalid IDs, duplicate IDs, missing fixture files, or invalid transcript schema.
- Each transcript step must define either `tool_name` or `finished`, never both.
- Keep `content_file` and `template_file` mutually exclusive.
- Use `content_file` for large payloads that belong next to the scenario file.
- Use `template_file` when the payload should come from `shared/agent_templates/`.

## Entry directories

- Each `entry_*` directory must include backend-owned `.manifests/current_role.json`.
- Keep scenario-local payload files inside the scenario directory tree.
- Let `tests/conftest.py` enforce the startup checks; do not work around them.

## Validation helpers

- Run `uv run python scripts/validate_integration_mock_response_preflight.py --scenario INT-###` after fixture edits.
- Use `--path` when you want to validate one scenario file or one `entry_*` directory.
- Use `--all` only when you intentionally want the whole corpus checked.
- Run `uv run python scripts/normalize_integration_mock_responses.py --fix ...` only for deterministic derived-field drift.

## Common drift to repair

- stale or missing `current_role.json`
- scenario files that no longer match the current `current_role.json` or latest handoff manifest
- malformed transcript steps
- missing `content_file` payloads
- bad derived totals or caps
- fixture edits that accidentally change model behavior instead of the intended boundary

## Do not

- Reuse another test's scenario data.
- Use fixtures to bypass freeze, mapping, scoring, or handoff gates.
- Broaden a fixture to cover an unrelated failure.
