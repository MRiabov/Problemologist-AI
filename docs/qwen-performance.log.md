# Qwen performance log

I've investigated how much Qwen spends on solving a task (here it failed, after 134 requests and 1.5 hours of work).
This shows that LLM performance is multiple times slower than system execution.

## Session Stats

### Interaction Summary

| Metric | Value |
| -- | -- |
| Session ID | `bb93f170-2274-48b3-894b-e0e395eb202f` |
| Tool Calls | 136 (✓ 134 × 2) |
| Success Rate | 98.5% |
| Code Changes | +967 -947 |

### Performance

| Metric | Value |
| -- | -- |
| Wall Time | 1h 36m 37s |
| Agent Active | 1h 4m 23s |
| » API Time | 56m 1s (87.0%) |
| » Tool Time | 8m 22s (13.0%) |
