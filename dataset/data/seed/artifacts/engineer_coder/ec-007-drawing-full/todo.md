# Implementation Checklist

- [ ] Read benchmark handoff package (`benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_script.py`)
- [ ] Read reviewed planner artifacts (`plan.md`, `assembly_definition.yaml`)
- [ ] Read drafting evidence (`solution_plan_evidence_script.py`, `solution_plan_technical_drawing_script.py`)
- [ ] Implement the low-friction cube passive chute in `solution_script.py`
- [ ] Verify inventory matches the approved planner multiset (labels, quantities, COTS identities)
- [ ] Run `validate_and_price` and confirm pass
- [ ] Run `simulate` and confirm goal-zone success
- [ ] Call `submit_for_review` with latest revision artifacts
- [ ] Update `todo.md` and `journal.md` during execution
