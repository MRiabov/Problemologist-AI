from typing import List
from build123d import Part, export_stl, Box


def export_mesh(
    part: Part, filepath: str, tolerance: float = 0.1, angular_tolerance: float = 0.1
) -> str:
    """
    Exports a build123d Part to an STL file.

    Args:
        part: The build123d Part to export.
        filepath: Path to the output STL file.
        tolerance: Linear deflection tolerance.
        angular_tolerance: Angular deflection tolerance.

    Returns:
        The path to the exported file.
    """
    export_stl(part, filepath, tolerance=tolerance, angular_tolerance=angular_tolerance)
    return filepath


def generate_colliders(part: Part) -> List[Part]:
    """
    Generates convex colliders for a given part.
    For the MVP, this returns the convex hull of the part if possible,
    otherwise it returns the bounding box as a simple collider.

    Args:
        part: The build123d Part to process.

    Returns:
        A list of Parts representing the convex colliders.
    """
    bbox = part.bounding_box()
    # Create a solid box from bounding box data
    collider = Box(bbox.size.X, bbox.size.Y, bbox.size.Z).translate(bbox.center())
    return [collider]
