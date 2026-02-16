# Agent Prompts & Handover Logic Review

**Date**: 2026-02-08  
**Files Reviewed**:

- `kitty-specs/desired_architecture.md`
- `config/prompts.yaml`

---

## ✅ What's Aligned

1. **Agent Structure**: Both correctly define `engineer` (planner/engineer/critic) and `benchmark_generator` (planner/coder/reviewer)
2. **Tools**: `validate_and_price`, `simulate`, `submit_for_review`, `get_docs_for` match
3. **Filesystem tools via deepagents**: `read/aread`, `write/awrite`, `edit/aedit`, `execute/aexecute`, `ls`
4. **Journal structure** (Intent/Result/Reflection/Next Step)
5. **Review YAML frontmatter** format
6. **`objectives.yaml`** structure matches

---

## ⚠️ Inconsistencies Found

| Issue | Architecture Says | Prompts.yaml Says |
|-------|------------------|------------------|
| **Benchmark coder output file** | `output.py` (line 245) | `result.py` (line 254) |
| **Engineer script name** | `script.py` (line 212) or custom | `solution.py` (line 69) |
| **Refuse plan function** | `refuse_plan` is a script that sends a request to an endpoint (lines 251, 265-266) | `refuse_plan(reason)` as a callable (line 96) - **not listed in utils imports** |
| **Validate vs validate_and_price** | Benchmark has `validate(Compound)` only (line 762), Engineer has `validate_and_price` | Benchmark coder prompt lists only `validate(compound)` ✓, but **template imports `validate_and_price`** in `common.code_template` (line 374) |
| **preview_design** | Only for Engineer (line 756) | Listed in Engineer prompt ✓, but benchmark coder doesn't have it (correct) |

---

## 🚨 Poor/Missing Business Logic Specification

### 1. Handover Contract Missing

Architecture (lines 443-478) describes handover flows in detail, but **prompts.yaml doesn't specify what data is passed between agents**:

- Planner → Coder/Engineer: Only mentions `plan.md` and `todo.md`, but not `objectives.yaml` creation flow
- The **benchmark coder must OUTPUT `objectives.yaml`** (architecture line 463), but there's no explicit handover specification for what the engineering planner receives

### 2. `refuse_plan` is Underspecified

- Where is it imported from? Not in `common.code_template`
- What's the return contract?
- How does Reviewer "confirm" the refusal (prompt line 98)?
<!-- addressed -->

### 3. Reviewer → Planner Routing Logic Missing

Architecture says "the Engineering Reviewer must agree... will pass the plan back to the lead engineer for replanning" (lines 453-454), but prompts don't specify the routing mechanism
<!-- addressed -->

### 4. Economic Constraints Handover

- **Architecture**: "Maximum prices and weight... estimated by the planner... 50% safety margin" (line 468)
- **But**: Benchmark Generator Coder prompt creates `max_unit_cost` in `objectives.yaml` (lines 234-236), not the Planner
- This contradicts architecture which implies the **benchmark planner sets these constraints**

### 5. Runtime vs Static Randomization

Prompts mention both but don't specify **who is responsible for generating each**:

- Static randomization → Benchmark coder generates variants?
- Runtime randomization → Specified in `objectives.yaml`, but **where does engineer get the specific jitter ranges?**

### 6. `to_mjcf` Function

In coder prompt (lines 209-216) - **not listed in utils imports or anywhere**

---

## 📋 Recommended Fixes

1. **Standardize file names**: Pick either `result.py` or `output.py` for benchmark, and `solution.py` or `script.py` for engineer
2. **Add missing imports** to `common.code_template`:
   - `refuse_plan`
   - `validate` (for benchmark coder)
   - `to_mjcf`
3. **Clarify economic constraints flow**: Benchmark planner should set `max_unit_cost`/`max_weight` in a draft `objectives.yaml`, then coder finalizes
4. **Document handover contracts explicitly**: What files are passed, who creates what
5. **Add routing logic** for refusals and rejections between agents
