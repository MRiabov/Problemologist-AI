# Phase 0: Research - Electromechanical Integration (WP3)

## PySpice & Ngspice Integration

- **Finding**: `libngspice.so` is available at `/usr/lib/x86_64-linux-gnu/libngspice.so`.
- **Validation**: Current implementation in `shared/pyspice_utils.py` correctly attempts to load this library.
- **Circuit Logic**: Motors are modeled as resistive loads ($R = V_{rated} / I_{stall}$). This is sufficient for DC power gating validation.
- **Recommendation**: Ensure `PYSPICE_LIBRARY_PATH` is set in the worker environment if the auto-discovery fails.

## 3D Wire Routing Coordination

- **Problem**: How to enable the "Electrical Agent" to route wires in a mechanical assembly without direct geometric manipulation?
- **Solution**: **Visual Coordination + Spline Query Tool**.
  - **Visual Map**: A depth-prebaked or proximity map of the assembly, identifying "Forbidden Volumes" and "Attachment Sites".
  - **Waypoints**: Agent proposes waypoints (X, Y, Z) based on the visual map.
  - **Spline Query Tool**: A tool that takes a set of waypoints and returns:
    - Total length.
    - Clearance status (Binary: Clear/Colliding).
    - Proximity to nearest collision (Distance in mm).
- **Implementation**: Use `build123d` for the distance query and `Polyline` or `Spline` for path representation.

## MuJoCo Tendons for Wires

- **Finding**: MuJoCo `spatial tendon` is ideal for wire simulation.
- **Physical Integrity**:
  - `tension`: Can be queried via `data.ten_length` or `data.ten_force`.
  - `breakage`: T015 implementation in `SimulationLoop` already monitors tension against AWG-rated limits.
- **Constraint**: MuJoCo tendons are "thin" lines. They don't have volume collision. We must rely on `build123d` clearance checks during the *design* phase to ensure wires don't clip through parts.

## COTS Electronics

- **Data Source**: `shared/cots/parts/electronics.py`.
- **Coverage**: Includes `PowerSupply`, `ElectronicRelay`, and `ServoMotor` (in `motors.py`).
- **Missing**: Terminal location metadata (X, Y, Z relative to part origin). Currently, the builder uses part origin as a site.
- **Recommendation**: Update `COTSPart` metadata to include `terminals` map (e.g., `{"v+": (x, y, z), "gnd": (x, y, z)}`).
