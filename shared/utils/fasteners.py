from enum import StrEnum

from bd_warehouse.fastener import (
    CounterSunkScrew,
    SocketHeadCapScrew,
)
from build123d import (
    Cone,
    Cylinder,
    Location,
    Part,
    RigidJoint,
)

# Standard metric coarse pitches for common sizes
STANDARD_PITCHES = {
    "M1.6": "0.35",
    "M2": "0.4",
    "M2.5": "0.45",
    "M3": "0.5",
    "M4": "0.7",
    "M5": "0.8",
    "M6": "1",
    "M8": "1.25",
    "M10": "1.5",
    "M12": "1.75",
}


class HoleType(StrEnum):
    FlatHeadHole = "FlatHeadHole"  # Uses CounterSink
    CounterBoreHole = "CounterBoreHole"  # Uses CounterBore
    SimpleHole = "SimpleHole"  # Uses simple Hole


def _get_fastener_instance(hole_type: HoleType, size: str, length: float):
    """Factory to get the appropriate fastener instance from bd-warehouse."""
    # Ensure size has pitch
    if "-" not in size:
        if size in STANDARD_PITCHES:
            size_with_pitch = f"{size}-{STANDARD_PITCHES[size]}"
        else:
            size_with_pitch = size
    else:
        size_with_pitch = size

    if hole_type == HoleType.FlatHeadHole:
        return CounterSunkScrew(size=size_with_pitch, length=length)
    if hole_type == HoleType.CounterBoreHole:
        return SocketHeadCapScrew(size=size_with_pitch, length=length)
    return SocketHeadCapScrew(size=size_with_pitch, length=length)


def fastener_hole(
    part: Part,
    pos: tuple[float, float] | tuple[float, float, float] | Location,
    depth: float,
    diameter: float,
    hole_id: str,
    hole_type: HoleType = HoleType.FlatHeadHole,
    add_fastener: bool = False,
    fit: str = "Normal",
) -> Part:
    """
    Creates a hole in the part for a fastener and assigns a RigidJoint.
    Aligned with desired_architecture.md parameters.

    Args:
        part: The part to modify.
        pos: Position (and optionally orientation) of the hole.
        depth: Depth of the fastener/hole (mm).
        diameter: Diameter of the fastener (mm).
        hole_id: Unique identifier for the joint.
        hole_type: Type of hole pattern.
        add_fastener: Whether to include the fastener visual.
        fit: Clearance fit ("Close", "Normal", "Loose").

    Returns:
        The modified part with the hole cut and RigidJoint assigned.
    """
    # Map diameter to size string (e.g. 3.0 -> "M3")
    size = f"M{int(diameter)}" if diameter == int(diameter) else f"M{diameter}"
    length = depth

    # Handle pos as Location or tuple
    if isinstance(pos, Location):
        location = pos
    elif len(pos) == 2:
        location = Location((pos[0], pos[1], 0))
    else:
        location = Location(pos)

    try:
        fastener = _get_fastener_instance(hole_type, size, length)
    except Exception as e:
        raise ValueError(f"Failed to create fastener for size '{size}': {e}") from e

    clearance_diam = fastener.clearance_hole_diameters[fit]
    radius = clearance_diam / 2.0

    tool = None

    if hole_type == HoleType.CounterBoreHole:
        cb_radius = (fastener.head_diameter / 2.0) + 0.2  # 0.2mm radial clearance
        cb_depth = fastener.head_height

        shank = Cylinder(radius=radius, height=100)
        shank = shank.move(Location((0, 0, -50)))

        head = Cylinder(radius=cb_radius, height=cb_depth)
        head = head.move(Location((0, 0, -cb_depth / 2)))

        tool = shank.fuse(head)

    elif hole_type == HoleType.FlatHeadHole:
        head_radius = fastener.head_diameter / 2.0
        cone_height = head_radius - radius
        if cone_height < 0:
            cone_height = 0

        shank = Cylinder(radius=radius, height=100)
        shank = shank.move(Location((0, 0, -50)))

        cone = Cone(bottom_radius=radius, top_radius=head_radius, height=cone_height)
        cone = cone.move(Location((0, 0, -cone_height / 2)))

        tool = shank.fuse(cone)
    else:
        tool = Cylinder(radius=radius, height=100)
        tool = tool.move(Location((0, 0, -50)))

    tool = tool.moved(location)
    new_part = part.cut(tool)

    if hasattr(part, "joints") and isinstance(part.joints, dict):
        new_part.joints = part.joints.copy()
    else:
        new_part.joints = {}

    new_part.joints[hole_id] = RigidJoint(label=hole_id, joint_location=location)

    return new_part
