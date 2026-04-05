from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    """Benchmark-owned fixture: sliding lift platform."""
    platform = Box(80.0, 160.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    platform = platform.move(Location((40.0, 0.0, 120.0)))
    platform.label = "lift_platform"
    platform.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)

    # Wrap in a labeled subassembly so the identity-pair validator counts it.
    subassembly = Compound(children=[platform])
    subassembly.label = "benchmark_fixtures"
    subassembly.metadata = CompoundMetadata()

    assembly = Compound(children=[subassembly])
    assembly.label = "benchmark_plan_evidence"
    assembly.metadata = CompoundMetadata()
    return assembly


def objectives_geometry() -> Compound:
    """Return the benchmark objective overlay geometry."""
    import yaml

    with open("benchmark_definition.yaml", encoding="utf-8") as fh:
        payload = yaml.safe_load(fh) or {}
    objectives = payload.get("objectives", {})

    children = []

    goal_zone = objectives.get("goal_zone")
    if isinstance(goal_zone, dict):
        bounds_min = goal_zone.get("min", [0.0, 0.0, 0.0])
        bounds_max = goal_zone.get("max", [0.0, 0.0, 0.0])
        size_x = float(bounds_max[0]) - float(bounds_min[0])
        size_y = float(bounds_max[1]) - float(bounds_min[1])
        size_z = float(bounds_max[2]) - float(bounds_min[2])
        center = (
            (float(bounds_min[0]) + float(bounds_max[0])) / 2.0,
            (float(bounds_min[1]) + float(bounds_max[1])) / 2.0,
            (float(bounds_min[2]) + float(bounds_max[2])) / 2.0,
        )
        zone = Box(
            max(size_x, 0.0),
            max(size_y, 0.0),
            max(size_z, 0.0),
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).move(Location(center))
        zone.label = "zone_goal"
        zone.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        children.append(zone)

    for index, forbid_zone in enumerate(objectives.get("forbid_zones", []) or []):
        if isinstance(forbid_zone, dict):
            bounds_min = forbid_zone.get("min", [0.0, 0.0, 0.0])
            bounds_max = forbid_zone.get("max", [0.0, 0.0, 0.0])
            size_x = float(bounds_max[0]) - float(bounds_min[0])
            size_y = float(bounds_max[1]) - float(bounds_min[1])
            size_z = float(bounds_max[2]) - float(bounds_min[2])
            center = (
                (float(bounds_min[0]) + float(bounds_max[0])) / 2.0,
                (float(bounds_min[1]) + float(bounds_max[1])) / 2.0,
                (float(bounds_min[2]) + float(bounds_max[2])) / 2.0,
            )
            zone = Box(
                max(size_x, 0.0),
                max(size_y, 0.0),
                max(size_z, 0.0),
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).move(Location(center))
            zone_name = (
                str(forbid_zone.get("name", f"forbid_{index}")).strip()
                or f"forbid_{index}"
            )
            zone.label = f"zone_forbid_{index}_{zone_name}"
            zone.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
            children.append(zone)

    build_zone = objectives.get("build_zone")
    if isinstance(build_zone, dict):
        bounds_min = build_zone.get("min", [0.0, 0.0, 0.0])
        bounds_max = build_zone.get("max", [0.0, 0.0, 0.0])
        size_x = float(bounds_max[0]) - float(bounds_min[0])
        size_y = float(bounds_max[1]) - float(bounds_min[1])
        size_z = float(bounds_max[2]) - float(bounds_min[2])
        center = (
            (float(bounds_min[0]) + float(bounds_max[0])) / 2.0,
            (float(bounds_min[1]) + float(bounds_max[1])) / 2.0,
            (float(bounds_min[2]) + float(bounds_max[2])) / 2.0,
        )
        zone = Box(
            max(size_x, 0.0),
            max(size_y, 0.0),
            max(size_z, 0.0),
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).move(Location(center))
        zone.label = "zone_build"
        zone.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
        children.append(zone)

    overlay = Compound(children=children) if children else Compound()
    overlay.label = "benchmark_objectives"
    overlay.metadata = CompoundMetadata()
    return overlay
