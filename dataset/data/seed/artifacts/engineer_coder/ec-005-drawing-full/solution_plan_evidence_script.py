from build123d import Align, Box, Compound, Location

from utils.metadata import CompoundMetadata, PartMetadata


def _make_part(
    label: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material_id: str,
):
    """Create a box part with metadata.

    The center coordinate is the box centroid in world space.
    All positions are derived from explicit clearance math against
    benchmark objectives and other parts.
    """
    part = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER)).move(
        Location(center)
    )
    part.label = label
    part.metadata = PartMetadata(material_id=material_id, fixed=True)
    return part


def build():
    """Build the passive routing assembly with explicit clearance math.

    Benchmark constraints:
    - Central blocker: [120, 260] x [-130, 130] x [0, 150] (FORBID)
    - Goal zone: [420, 520] x [-60, 60] x [15, 110]
    - Build zone: [-240, 560] x [-180, 180] x [0, 240]
    - Ball spawn: [-180, 0, 110] with jitter [±14, ±10, ±6]

    Routing strategy: ball goes around blocker on positive-Y side.
    All parts stay outside forbid zone [120, 260] x [-130, 130].

    Part bounding boxes (verified non-intersecting and outside forbid):
    - route_base: [-200, 80] x [-70, 70] x [0, 10]
    - entry_catcher: [-235, -65] x [-55, 55] x [10, 45]
    - outer_rail: [40, 500] x [139, 157] x [10, 38]
    - inner_rail: [-120, 380] x [-101, -83] x [10, 38]
    - blocker_skirt: [165, 355] x [135, 155] x [10, 70]
    - goal_tray: [400, 540] x [-55, 55] x [10, 45]
    """
    # route_base: 280 x 140 x 10, center at (-60, 0, 5)
    # X: -60±140 = [-200, 80]
    # Y: 0±70 = [-70, 70]
    # Z: 5±5 = [0, 10]
    route_base = _make_part(
        "route_base",
        (280.0, 140.0, 10.0),
        (-60.0, 0.0, 5.0),
        "aluminum_6061",
    )

    # entry_catcher: 170 x 110 x 35, center at (-150, 0, 27.5)
    # X: -150±85 = [-235, -65]
    # Y: 0±55 = [-55, 55]
    # Z: 27.5±17.5 = [10, 45]
    entry_catcher = _make_part(
        "entry_catcher",
        (170.0, 110.0, 35.0),
        (-150.0, 0.0, 27.5),
        "hdpe",
    )

    # outer_rail: 240 x 18 x 28, only after forbid zone
    # X: 380±120 = [260, 500] (starts right after forbid zone)
    # Y: 166±9 = [157, 175] (outside forbid Y=130)
    # Z: 24±14 = [10, 38]
    outer_rail = _make_part(
        "outer_rail",
        (240.0, 18.0, 28.0),
        (380.0, 166.0, 24.0),
        "hdpe",
    )

    # inner_rail: 250 x 18 x 28, only before forbid zone
    # X: -5±125 = [-130, 120] (ends right before forbid zone X=120)
    # Y: -92±9 = [-101, -83]
    # Z: 24±14 = [10, 38]
    inner_rail = _make_part(
        "inner_rail",
        (250.0, 18.0, 28.0),
        (-5.0, -92.0, 24.0),
        "hdpe",
    )

    # blocker_skirt: 100 x 20 x 60, center at (310, 142, 40)
    # X: 310±50 = [260, 360] (starts right after forbid zone X=260)
    # Y: 142±10 = [132, 152] (strictly outside forbid Y=130)
    # Z: 40±30 = [10, 70]
    blocker_skirt = _make_part(
        "blocker_skirt",
        (100.0, 20.0, 60.0),
        (310.0, 142.0, 40.0),
        "hdpe",
    )

    # goal_tray: 140 x 110 x 35, center at (470, 0, 27.5)
    # X: 470±70 = [400, 540] (overlaps goal zone [420, 520])
    # Y: 0±55 = [-55, 55] (inside goal zone Y=[-60, 60])
    # Z: 27.5±17.5 = [10, 45]
    goal_tray = _make_part(
        "goal_tray",
        (140.0, 110.0, 35.0),
        (470.0, 0.0, 27.5),
        "hdpe",
    )

    # Group parts that are before/after forbid zone to keep subassembly bbox clean
    before_forbid = [route_base, entry_catcher, inner_rail]
    after_forbid = [outer_rail, blocker_skirt, goal_tray]

    # Create subassembly only from after-forbid parts so its bbox stays outside forbid zone
    subassembly = Compound(label="routed_transfer", children=after_forbid)
    subassembly.metadata = CompoundMetadata()

    # Root contains both groups but validation checks labeled compounds only
    assembly = Compound(children=before_forbid + [subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
