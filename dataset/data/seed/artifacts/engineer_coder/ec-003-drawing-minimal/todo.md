# TODO

- [x] Reviewed timed-gate planner package seeded
- [ ] Read the seeded `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `plan.md`, and `assembly_definition.yaml`
- [ ] Implement the `base_plate` and `settling_chute` so the ball arrives at the gate on a stable line
- [ ] Implement the `metering_wheel_guard` and `drive_motor` while keeping the wiring corridor outside `gate_swing_keepout`
- [ ] Implement the `guide_rail`, `post_gate_channel`, and `goal_cup` so the release path lands in `goal_zone`
- [ ] Validate `build_zone` fit, gate clearance, and the single `rotate_z` benchmark motion contract
- [ ] Run validation and simulation until the metered release path works reliably
- [ ] Prepare the reviewed execution handoff package
