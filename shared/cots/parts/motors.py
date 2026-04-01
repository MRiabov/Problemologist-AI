from build123d import Align, Box, Color, Cylinder, Location

from shared.cots.base import COTSPart


def _resolve_motor_identity(size: str) -> tuple[str, str]:
    """Normalize a direct constructor argument to a catalog id and motor variant."""
    raw = str(size).strip()
    if not raw:
        raise ValueError("size must be a non-empty string")

    if raw in ServoMotor.motor_data:
        variant = raw
        return f"ServoMotor_{variant}", variant

    if raw.startswith("ServoMotor_"):
        variant = raw.split("ServoMotor_", 1)[1].strip()
        if variant in ServoMotor.motor_data:
            return raw, variant
        raise ValueError(f"Unknown servo motor variant '{variant}'")

    raise ValueError(f"Unknown servo motor size or catalog id '{size}'")


class ServoMotor(COTSPart):
    """
    Represents a COTS servo motor.
    Mimics bd-warehouse part structure for Indexer compatibility.
    """

    # Data for common servos
    # Torque in N-m, Speed in sec/60deg, Weight in g
    # Dimensions in mm (L, W, H)
    # Price in USD (approx)
    failed = False  # Check if initialization failed

    motor_data = {
        "SG90": {
            "torque_nm": 0.176,  # ~1.8 kg-cm
            "speed_sec_60deg": 0.1,
            "weight_g": 9.0,
            "dims": (22.2, 11.8, 31.0),  # Approx body size
            "price": 2.50,
            "voltage": 4.8,
            "stall_current": 0.65,
        },
        "MG996R": {
            "torque_nm": 0.98,  # ~10 kg-cm
            "speed_sec_60deg": 0.17,
            "weight_g": 55.0,
            "dims": (40.7, 19.7, 42.9),
            "price": 12.00,
            "voltage": 4.8,
            "stall_current": 2.5,
        },
        "DS3218": {
            "torque_nm": 1.96,  # ~20 kg-cm @ 6.8V
            "speed_sec_60deg": 0.16,
            "weight_g": 60.0,
            "dims": (40.0, 20.0, 40.5),  # standard size
            "price": 18.00,
            "voltage": 6.8,
            "stall_current": 3.0,
        },
    }

    # Alias for Indexer compatibility (indexer checks fastener_data or bearing_data)
    # We'll patch indexer to check motor_data too, but just in case:
    fastener_data = motor_data

    def __init__(self, size: str = "SG90", label: str | None = None, **kwargs):
        """
        Initialize the servo motor geometry and metadata.
        Args:
            size: The model name (e.g. "SG90")
            kwargs: implementation specifics
        """
        catalog_part_id, variant = _resolve_motor_identity(size)
        data = self.motor_data[variant]
        l, w, h = data["dims"]
        shaft_height = max(h * 0.18, 5.0)
        shaft_radius = max(min(l, w) * 0.06, 1.8)
        cable_depth = max(w * 0.3, 5.0)
        cable_length = max(l * 0.18, 6.0)
        cable_height = max(h * 0.08, 3.0)

        # Create an interface-faithful proxy: body, shaft stub, and cable exit.
        body = Box(
            l,
            w,
            h,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        body.color = Color("black")
        shaft = Cylinder(
            radius=shaft_radius,
            height=shaft_height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).move(Location((0, 0, h)))
        shaft.color = Color("silver")
        cable_exit = Box(
            cable_length,
            cable_depth,
            cable_height,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        ).move(Location((0, -(w / 2 + cable_depth / 2), max(h * 0.12, 3.0))))
        cable_exit.color = Color("dimgray")

        # COTSPart handles children, metadata, label, and emission.
        super().__init__(
            category="motor",
            part_number=catalog_part_id,
            data=data,
            children=[body, shaft, cable_exit],
            label=label,
        )
        if label is None:
            self.label = f"motor_{variant}"
        self.motor_variant = variant
        self.catalog_part_id = catalog_part_id

    @property
    def volume(self):
        # Return volume of the bounding box for simplicity if exact geometry is box
        bb = self.bounding_box()
        return bb.size.X * bb.size.Y * bb.size.Z

    @property
    def info(self):
        """Return dict of properties."""
        return self.metadata


def retrieve_cots_physics(cots_id: str) -> dict[str, float] | None:
    """
    Retrieve physics parameters for a COTS part by ID.
    Returns None if ID not found or not a motor.
    """
    try:
        _, variant = _resolve_motor_identity(cots_id)
    except ValueError:
        return None

    data = ServoMotor.motor_data[variant]
    if data:
        torque = data["torque_nm"]

        # heuristic for KP/KV if not specified in DB
        # KP = torque / saturation_error (rad) ~ 0.2 rad
        saturation_error = 0.2
        kp = torque / saturation_error
        # KV = KP * 0.1 (critical damping approx)
        kv = kp * 0.1
        return {
            "torque": torque,
            "kp": kp,
            "kv": kv,
            "max_velocity": 60.0 / data["speed_sec_60deg"],
        }
