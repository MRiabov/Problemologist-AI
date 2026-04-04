# TODO

- [x] Reviewed timed-gate planner package seeded
- [ ] Read the seeded `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `plan.md`, and `assembly_definition.yaml`
- [ ] Encode the coarse `motion_forecast` in `assembly_definition.yaml` as the build-safe start-to-goal trajectory
- [ ] Narrow the same moving-part path in `payload_trajectory_definition.yaml` without changing the moving-part set
- [ ] Implement the `base_plate` and `settling_chute` so the ball arrives at the gate on a stable line
- [ ] Implement the `metering_wheel_guard` and `drive_motor` while keeping the wiring corridor outside `gate_swing_keepout`
- [ ] Implement the `guide_rail`, `post_gate_channel`, and `goal_cup` so the release path lands in `goal_zone`
- [ ] Validate `build_zone` fit, gate clearance, and the staged `motion_forecast` / `payload_trajectory_definition.yaml` endpoint contract
- [ ] Run validation and simulation until the metered release path works reliably
- [ ] Prepare the reviewed execution handoff package
