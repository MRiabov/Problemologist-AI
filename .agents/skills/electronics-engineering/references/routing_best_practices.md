# 3D Wire Routing Best Practices

## 1. Route For Physics, Not Just Topology

- Start and end waypoints should sit near real terminals or attachment points.
- Route along frames, walls, or other support geometry when possible.
- A logical connection with no path is acceptable only if the task does not require physical wire behavior.

## 2. Clearance

- Treat `2 mm` as the baseline static clearance because that is the default `check_wire_clearance(...)` contract.
- Give extra room around moving parts and sweep volumes.
- Do not route through hinges, sliders, belts, or likely contact corridors.

## 3. Slack And Motion

If a wire connects to a moving part:

- add service slack with intermediate waypoints
- avoid taut straight-line routing
- keep bend changes gradual enough that motion does not instantly spike tension

## 4. Gauge Selection

Use gauge as a current-carrying decision, not decoration.

- `18-20 AWG`: small DC motor loads
- `14-16 AWG`: higher-current or longer runs
- `22-26 AWG`: low-current signal paths

Then confirm with electrical validation rather than trusting the heuristic alone.

## 5. Review Failure Patterns

Reject these routes:

- path intersects assembly geometry away from attachment points
- route crosses a moving mechanism envelope
- wire to a moving part has no slack
- very short zig-zag segments that imply unstable routing
