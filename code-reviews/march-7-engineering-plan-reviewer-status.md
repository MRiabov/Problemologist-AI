# Engineering Plan Reviewer Implementation Review - March 7, 2026

## Executive Summary

The "Engineering Plan Reviewer" (EPR) node is functional within the LangGraph orchestration but lacks several critical "deterministic" features mandated by the recent architecture specifications. While it correctly handles basic file handovers and plan refusal loops, it does not yet implement the manifest-based validation or persistent review storage required for production-grade robustness and steerability.

## Implementation Status vs. Desired Architecture

| Feature Area           | Status              | Gaps / Observations                                                                                                                                               |
| :--------------------- | :------------------ | :---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Handoff Artifacts**  | Partial (80%)       | Correctly requires `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`. Missing - `manifests/engineering_plan_review_manifest.json` |
| **Manifest System**    | **Missing (0%)**    | Spec requires `.manifests/engineering_plan_review_manifest.json`. Currently non-existent.                                                                         |
| **Review Persistence** | **Missing (0%)**    | Spec requires reviews to be persisted in `reviews/review-round-*/` for long-term agent memory.                                                                    |
| **Node Entry Gates**   | Partial (70%)       | Artifact checks are in place, but manifest validation and stale-revision checks are missing.                                                                      |
| **Refusal Handling**   | **Complete (100%)** | Correctly reads `plan_refusal.md` and routes back to Planner or Coder based on decision.                                                                          |
| **Cost/Weight Gates**  | Partial (50%)       | `validate_and_price.py` exists but is not explicitly called by the Reviewer for verification.                                                                     |
| **DOF Minimization**   | Partial (20%)       | Requirement is in the prompt, but no explicit/tool-assisted validation is implemented.                                                                            |

<!--I'm reviewing the analysis-->

## Human review of above:

**Review Persistence**: Should be persisted as reviews/plan-review-round-\*.md; not as a folder nor as a single name for all. We currently have reviewers files and I insist on keeping naming distinctive.
**Cost/Weight Gates**: Ouch. I'm not exactly sure if reviewer or even planner can call tools like execute_command which would allow it to run python scripts. I think I removed it from them. Or maybe not. Anyway, this is necessary, if not, edit `agents_config.yaml` and instruct them to call the tools.
**DOF Minimization**: DOF minimization may be implemented as any DOF > 3 and it might highlight as an input addition to plan reviewer prompt, based on rule-based analysis. I.e. we'll scan a YAML for DOFs>3 and say something like "these have suspiciously too much".

## Detailed Gap Analysis

### 1. Deterministic Manifests

The architecture (`specs/architecture/agents/handover-contracts.md`) mandates a `engineering_plan_review_manifest.json`.

- **Current State:** The system relies on the existence of files but does not verify if they correspond to the "latest revision" via hashes in a manifest.
- **Risk:** Stale plans or mismatched artifacts could be passed to the Coder if the implementation node fails to refresh them.

### 2. Review Storage & Context

The spec states: *"The goal is to persist the reviews into a persistent file which the agent can reference at any time"*.

- **Current State:** Reviews exist only in `AgentState.feedback` and `AgentState.journal`.
- **Risk:** As the conversation length increases, older review details may be truncated or "forgotten" by the LLM, leading to recurring issues in re-planning loops.

### 3. Automated Validation Tools

The Reviewer is currently an "LLM-as-judge" role without programmatic teeth.

- **Current State:** It reads `assembly_definition.yaml` but does not re-run `validate_and_price.py` to ensure the Planner didn't fudge the numbers.
- **Requirement:** The EPR should ideally use the same validation tools as the Planner to confirm budget/geometry claims.

### 4. DOF Minimization Gate

A key recent architectural focus is the minimization of Degrees of Freedom (`final_assembly.parts[*].dofs`).

- **Current State:** There is no specific tool for the EPR to detect "unjustified" DOFs other than the LLM's raw reasoning.
- **Recommendation:** Implement a simple check or specific prompt section that extracts DOFs for explicit approval/rejection.

## Actionable Recommendations

1. **Implement Manifest Generation:** Update `PlanReviewerNode` to write a manifest upon decision.
2. **Implement Review Persistence:** Create a utility to write `ReviewResult` to the worker filesystem under `reviews/review-round-X/`.
3. **Enhance Node Entry Contract:** Add a `custom_check` to `ENGINEER_PLAN_REVIEWER` in `node_entry_validation.py` that mirrors the logic used for the Execution Reviewer.
4. **Integrate Validation Scripts:** Allow the Reviewer to call `validate_and_price.py` (or its underlying logic) to verify the Planner's `assembly_definition.yaml` data.

## Conclusion

We have a solid foundation with the Graph structure and the basic node implementation. However, to meet the "Desired Architecture" standards for March 2026, we must move from a purely message-based review system to the specified file-based manifest and persistence architecture.
