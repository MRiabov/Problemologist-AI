import json
from build123d import Box, Compound
from shared.models.schemas import ElectronicsSection, WireConfig, WireTerminal, PowerSupplyConfig
from worker.simulation.builder import GenesisSimulationBuilder

def test_genesis_builder_electronics(tmp_path):
    # Setup assembly
    part1 = Box(0.1, 0.1, 0.1)
    part1.label = "part1"
    part2 = Box(0.1, 0.1, 0.1)
    part2.label = "part2"
    assembly = Compound(children=[part1, part2])

    # Setup electronics
    wire = WireConfig(
        wire_id="wire1",
        from_terminal=WireTerminal(component="part1", terminal="v+"),
        to_terminal=WireTerminal(component="part2", terminal="v-"),
        gauge_awg=20,
        length_mm=100.0,
        routed_in_3d=True,
        waypoints=[(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
    )
    electronics = ElectronicsSection(
        power_supply=PowerSupplyConfig(voltage_dc=12, max_current_a=10),
        wiring=[wire],
        components=[]
    )

    builder = GenesisSimulationBuilder(tmp_path)
    scene_path = builder.build_from_assembly(assembly, electronics=electronics)

    # Assert JSON content
    with open(scene_path) as f:
        data = json.load(f)
        assert "cables" in data
        assert len(data["cables"]) == 1
        cable = data["cables"][0]
        assert cable["name"] == "wire1"
        assert cable["radius"] > 0
        assert len(cable["points"]) == 2
        assert cable["points"][0]["type"] == "world"
