from shared.circuit_builder import build_circuit_from_section
from shared.models.schemas import (
    ElectronicsSection,
    PowerSupplyConfig,
    ElectronicComponent,
    WireConfig,
    WireTerminal,
)
from shared.enums import ElectronicComponentType


def test_connector_gap():
    # Setup
    # Create a section with a Connector
    psu = PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0)

    connector = ElectronicComponent(
        component_id="conn1", type=ElectronicComponentType.CONNECTOR
    )

    # Simple wiring: PSU+ -> Conn1_1, Conn1_2 -> PSU- (Short circuit if Conn1 connects 1-2, but it doesn't by default)
    # Just checking if builder accepts it.

    section = ElectronicsSection(power_supply=psu, components=[connector], wiring=[])

    # Execute
    try:
        circuit = build_circuit_from_section(section)
        print("Circuit built successfully.")
        print(f"Elements: {list(circuit.element_names)}")

        # Verify if connector is present as an element
        # Current behavior: It is ignored, so no element "conn1" or similar.
        has_connector_element = any("conn1" in str(e) for e in circuit.element_names)
        print(f"Has connector element: {has_connector_element}")

        # Assert that the connector is NOT created as a circuit element
        # This confirms our design choice to treat it as a passive node junction.
        assert not has_connector_element, (
            "Connector should be treated as a passive node, not a circuit element."
        )

        # Also assert that the PSU IS created
        assert "Vsupply" in circuit.element_names, "Power supply should be created."

    except Exception as e:
        print(f"Builder crashed: {e}")
        raise


if __name__ == "__main__":
    test_connector_gap()
