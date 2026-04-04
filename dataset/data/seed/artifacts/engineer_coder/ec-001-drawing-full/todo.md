# TODO

- [x] Planner handoff package seeded
- [ ] Read `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `plan.md`, `assembly_definition.yaml`, and `payload_trajectory_definition.yaml`
- [ ] Implement the freestanding `base_plate` and `entry_funnel` in `solution_script.py`
- [ ] Implement the driven `roller_bed` and `idler_guide` with the planned clearances
- [ ] Implement the `goal_tray` so it overlaps the seeded goal zone
- [ ] Preserve the seeded `ServoMotor_DS3218` mount and left-side cable corridor assumptions in `solution_script.py`
- [ ] Validate the coarse `motion_forecast` and the narrowed `payload_trajectory_definition.yaml` against the build-safe start and goal-contact finish
- [ ] Validate build-zone fit, exact inventory grounding, and the new detailed calculations
- [ ] Run validation and simulation until the ball reaches the goal robustly
- [ ] Prepare the reviewed execution handoff package
