# Runtime Script Function Signatures

This is a generated snapshot of the current submission-helper surface.
Refresh it from the live helper module when signatures change; do not hand-edit arities from memory.

## Submission Helpers

- `validate_benchmark(compound: Compound, **kwargs) -> tuple[bool, str | None]`
- `validate_engineering(compound: Compound, **kwargs) -> tuple[bool, str | None]`
- `simulate_benchmark(compound: Compound, **kwargs) -> BenchmarkToolResponse`
- `simulate_engineering(compound: Compound, **kwargs) -> BenchmarkToolResponse`
- `submit_benchmark_for_review(compound: Compound) -> bool`
- `submit_solution_for_review(compound: Compound) -> bool`

## Related Preview Signatures

The preview, bundle-history, and point-pick helpers live in [render-evidence function signatures](../../render-evidence/references/function_signatures.md).
