# TODO

- [x] Planner-reviewed handoff package seeded
- [ ] Read the seeded benchmark and engineering planner artifacts, including `assembly_definition.yaml.motion_forecast` and `precise_path_definition.yaml`
- [ ] Check `plan.md` detailed calculations against `benchmark_definition.yaml`
- [ ] Confirm the bridge deployment stays gap-clear and grounded in the seeded fixtures
- [ ] Encode the coarse `motion_forecast` in `assembly_definition.yaml` as the bridge-deck deployment trajectory
- [ ] Narrow the same `bridge_deck` motion in `precise_path_definition.yaml` without changing the moving-part set
- [ ] Implement `base_frame`, `bridge_deck`, `left_fence`, `right_fence`, and `landing_pocket` in `solution_script.py`
- [ ] Preserve the benchmark-owned `transfer_cube` and read-only fixture context
- [ ] Validate build-zone fit, inventory exactness, and planner budget margins
- [ ] Prepare the reviewed execution handoff package after implementation
