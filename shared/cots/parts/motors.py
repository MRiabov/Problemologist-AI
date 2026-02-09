from build123d import Box, Color, Compound, Location, Part


class ServoMotor(Compound):
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

    def __init__(self, size: str = "SG90", **kwargs):
        """
        Initialize the servo motor geometry and metadata.
        Args:
            size: The model name (e.g. "SG90")
            kwargs: implementation specifics
        """
        if size not in self.motor_data:
            # Fallback or error
            self.failed = True
            # Create dummy object to avoid crash during indexing if iterated
            super().__init__(children=[])
            return

        data = self.motor_data[size]
        l, w, h = data["dims"]

        # Create a simple box representation
        # Body
        body = Box(l, w, h)
        # Color it black/dark
        body.color = Color("black")

        super().__init__(children=[body])
        self.label = f"Servo_{size}"

        # Metadata storage
        self.metadata = data.copy()
        self.metadata["part_number"] = size

    @property
    def volume(self):
        # Return volume of the bounding box for simplicity if exact geometry is box
        bb = self.bounding_box()
        return bb.size.X * bb.size.Y * bb.size.Z

    @property
    def info(self):
        """Return dict of properties."""
        return self.metadata
