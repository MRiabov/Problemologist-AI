# ec-001 Run Hints

Quick reference from the most useful prior `ec-001` runs.

## Baseline Geometry

- `run_20260330_073100`: simplest working layout shape.
- Key pattern:
  - `base_plate`: `980 x 170 x 10`
  - `entry_funnel`: `180 x 140 x 40`
  - `roller_bed`: `780 x 70 x 28`
  - `idler_guide`: `780 x 18 x 24`
  - `goal_tray`: `150 x 120 x 35`
  - `ServoMotor_DS3218`: parked left of the lane at about `(-120, -58, 37.5)`

## Iteration Notes

- `run_20260330_105147`:

  - Switched to loft-based geometry and added helper functions.
  - Useful note: the lane should drain toward positive X, not away from it.
  - Keep the fit-safe layout inside the 1000 mm envelope: the funnel should sit closer to `x=-400` mm and the goal tray closer to `x=430` mm, not `x=485` mm.
  - Quick fit check: do not let any child extend past roughly `x=-500` on the left or `x=+500` on the right.

- `run_20260330_111506`:

  - Tried shell-box geometry with a shared tilt.
  - Useful note: the slope sign mattered; a left-draining slope sent the ball the wrong way.
  - The upper guide likely sat too close to the lane and needed more clearance.

- `run_20260330_113210`:

  - Geometry was revised again, but the blocker looked like a validation/render path issue rather than a brand-new layout idea.
  - Useful note: do not churn geometry if the submission helper is failing for environment reasons.
  - The earlier failed session never produced `.manifests/engineering_execution_handoff_manifest.json`; make sure `bash scripts/submit_for_review.sh` runs all the way through the handoff step.

## Contract Reminders

- Use `utils.metadata`, not `shared.models.schemas`, in the authored workspace script.
- Keep the assembly import-safe: expose `result = build()` or `build() -> Compound`.
- Preserve the left-side motor/wiring corridor and keep the goal tray overlapping the goal zone start at `x=430 mm`.
- Treat `assembly_definition.yaml` as the coarse `motion_forecast` for `transfer_lane`, then narrow the same moving-part set in `precise_path_definition.yaml` without renaming or adding motion-bearing parts.
- For catalog-backed motor geometry, use `ServoMotor.from_catalog_id("ServoMotor_DS3218")` so the seed path matches the class-aware factory contract.
- Prefer the simplest static compound that satisfies the planner handoff before adding extra mechanism details.
- Do not stop at local validation/simulation; the run is incomplete until `bash scripts/submit_for_review.sh` writes `.manifests/engineering_execution_handoff_manifest.json`.
