# Agent Evaluations Review (Feb 10, 2026)

## Scope

Focus: `desired_architecture.md` section "Evaluations" (~line 676). This review only addresses agent evaluations for *ability to solve problems*, not system-wide or business-goal coverage.

## Summary

Current evals emphasize validity (YAML/markdown/lint/manufacturability) and constraint compliance, but they do not directly measure *problem-solving success* (goal achievement in simulation), nor recovery, robustness, or plan-to-implementation fidelity. As written, they are incomplete for evaluating agent problem-solving ability.

## Gaps

1. End-to-end solve rate is missing (goal reached in simulation).
2. Solve rate under full constraints is missing (goal + cost/weight/build zone simultaneously).
3. Recovery behavior is missing (agent fixes failure within bounded attempts).
4. Robustness to randomization is missing (performance across seeds/variants).
5. Plan-to-CAD fidelity is missing (engineer implements intended mechanism).
6. Reviewer-feedback efficacy is missing (solution improves after review).

## Recommended Additions (Agent-Solve Focused)

1. Engineer end-to-end success rate: goal reached in simulation within N attempts and M tool calls.
2. Constrained success: goal reached while satisfying `max_unit_cost`, `max_weight`, and build zone in >= X% cases.
3. Iterative recovery: if first attempt fails, success by attempt 2 or 3 in >= Y% cases.
4. Robustness to randomization: success across K randomized seeds/variants.
5. Plan-to-implementation fidelity: CAD output aligns with planner mechanism in >= X% cases.
6. Reviewer feedback loop: after review, engineer improves and succeeds in >= X% cases.

## Notes on Existing Items

- CAD Engineer manufacturability checks are duplicated and underspecified.
- Reviewer forcing cheaper solutions lacks criteria for when it is appropriate or correct.
- Planner validation via LLM-as-judge exists but is not tied to actual solve success.
