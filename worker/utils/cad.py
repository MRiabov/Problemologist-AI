from enum import Enum

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


class HoleType(Enum):
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
            # Fallback for unknown sizes, assume user might have provided full string or let bd-warehouse error
            # If standard pitch logic fails, we try to guess based on common usage or fail
            # For now, let's just pass it through if not found, but warn?
            # bd-warehouse might default or error.
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
    location: Location,
    hole_id: str,
    size: str = "M3",
    length: float = 10.0,
    hole_type: HoleType = HoleType.CounterBoreHole,
    add_fastener: bool = False,
    fit: str = "Normal",
) -> Part:
    """
    Creates a hole in the part for a fastener and assigns a RigidJoint.

    Args:
        part: The part to modify.
        location: The location (position + orientation) of the hole/joint.
        hole_id: Unique identifier for the joint.
        size: Fastener size (e.g., "M3", "M4"). Pitch is auto-appended if missing.
        length: Length of the fastener (used for validation/selection).
        hole_type: Type of hole pattern.
        add_fastener: (Not fully implemented validation) - intended to signal fastener addition.
        fit: Clearance fit ("Close", "Normal", "Loose").

    Returns:
        The modified part with the hole cut and RigidJoint assigned.
    """
    try:
        fastener = _get_fastener_instance(hole_type, size, length)
    except Exception as e:
        # Fallback for invalid sizes or errors, return primitive hole info or raise
        raise ValueError(f"Failed to create fastener for size '{size}': {e}") from e

    # Determine hole dimensions
    clearance_diam = fastener.clearance_hole_diameters[fit]
    radius = clearance_diam / 2.0

    # Perform the boolean operation using build123d Context or direct algebra
    # We will use the 'part - feature' approach by creating the feature at the location

    # We create the hole Feature as a Solid/Part and subtract it.
    # However, build123d's CounterBoreHole/CounterSinkHole are operations within a context.
    # We can create a temporary object, apply the hole, and extract the negative volume?
    # Or just subtract a custom-built shape.

    # Simple shape approach is robust:
    tool = None

    if hole_type == HoleType.CounterBoreHole:
        cb_radius = (fastener.head_diameter / 2.0) + 0.2  # 0.2mm radial clearance
        cb_depth = fastener.head_height

        # Create a tool: Cylinder for shank + Cylinder for head
        # Shank goes through. Length needs to be sufficient to cut through the part at that location.
        # We assume 'length' or just a large depth?
        # The hole usually goes 'through' or to a depth.
        # 'length' arg is fastener length. The hole might need to be deeper or through.
        # For now, let's use a reasonable depth (e.g. 2x length or fixed large value if 'through')
        # BUT 'fastener_hole' implies adapting to the part.
        # If we use `Part` directly, we might not know the depth needed.
        # SimpleHole in build123d implies 'through everything' in the context.
        # When working with Part objects explicitly, we need to define the tool size.

        # Let's use a "long enough" cutter, centered? Or starting from location going -Z?
        # Fastener location usually implies head is at Z=0 (or surface), pointing -Z.

        shank = Cylinder(radius=radius, height=100)  # Arbitrary long length
        shank = shank.move(Location((0, 0, -50)))  # Move so top is at 0

        head = Cylinder(radius=cb_radius, height=cb_depth)
        head = head.move(
            Location((0, 0, -cb_depth / 2))
        )  # Head sits below Z=0 surface?
        # Wait, Counterbore means head is IN the material.
        # So top of head is at Z=0.

        tool = shank.fuse(head)
        # However, boolean union of touching solids might be tricky? defaults to Fuse.

    elif hole_type == HoleType.FlatHeadHole:
        # Countersink (Cone + Cylinder)
        head_radius = fastener.head_diameter / 2.0
        # Angle usually 90 degrees for metric.
        # Height of cone = (head_radius - radius) / tan(45) = head_radius - radius
        cone_height = head_radius - radius
        if cone_height < 0:
            cone_height = 0  # Should not happen if head > shank

        shank = Cylinder(radius=radius, height=100)
        shank = shank.move(Location((0, 0, -50)))

        cone = Cone(bottom_radius=radius, top_radius=head_radius, height=cone_height)
        cone = cone.move(Location((0, 0, -cone_height / 2)))
        # Cone top is at Z=0.

        tool = shank.fuse(cone)

        # HUMAN DEVELOPER REVIEW: why this? it should be a Build123d holes or bd-warehouse fasteners, not... custom code. It could be, but why bother?

    else:
        # Simple hole
        tool = Cylinder(radius=radius, height=100)
        tool = tool.move(Location((0, 0, -50)))

    # Move tool to the specific location
    # Note: location defines the 'surface' point and orientation (Z-axis is hole axis into material)
    tool = tool.moved(location)

    # Boolean cut
    new_part = part.cut(tool)

    # Assign RigidJoint
    # We must ensure to copy existing joints if any
    if hasattr(part, "joints") and isinstance(part.joints, dict):
        new_part.joints = part.joints.copy()
    else:
        new_part.joints = {}

    new_part.joints[hole_id] = RigidJoint(label=hole_id, joint_location=location)

    return new_part
