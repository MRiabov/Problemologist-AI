from build123d import *  # noqa: F403

from shared.models.schemas import ComponentConfig, ElectronicsSection, PowerSupplyConfig
from shared.wire_utils import route_wire

# 1. Define Electronics Section
electronics = ElectronicsSection(
    power_supply=PowerSupplyConfig(voltage_dc=24.0, max_current_a=10.0),
    components=[
        ComponentConfig(
            component_id="motor_1",
            type="motor",
            rated_voltage=24.0,
            stall_current_a=2.5,
        ),
        ComponentConfig(component_id="relay_1", type="relay"),
    ],
    wiring=[],
)

# 2. Route Wires in 3D
# Waypoints should avoid moving parts and maintain clearance
w1_path = [(0, 0, 0), (10, 5, 0), (20, 0, 0)]
w1 = route_wire(
    wire_id="w1",
    from_comp="supply",
    from_term="v+",
    to_comp="relay_1",
    to_term="in",
    gauge_awg=18,
    waypoints=w1_path,
    routed_in_3d=True,
)
electronics.wiring.append(w1)

# 3. Validation Logic (performed by simulation)
# is_powered("motor_1", t) will gating the torque based on circuit state.
