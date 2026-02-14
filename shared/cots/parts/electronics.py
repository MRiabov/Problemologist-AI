from build123d import Box, Color

from shared.cots.base import COTSPart


class PowerSupply(COTSPart):
    """
    Represents a COTS DC Power Supply.
    """

    psu_data = {
        "LRS-350-24": {
            "voltage_dc": 24.0,
            "max_current_a": 14.6,
            "weight_g": 760.0,
            "dims": (215.0, 115.0, 30.0),
            "price": 35.00,
        },
        "DR-120-24": {
            "voltage_dc": 24.0,
            "max_current_a": 5.0,
            "weight_g": 790.0,
            "dims": (65.5, 125.2, 100.0),
            "price": 45.00,
            "mounting": "DIN-rail",
        },
        "GST60A24-P1J": {
            "voltage_dc": 24.0,
            "max_current_a": 2.5,
            "weight_g": 305.0,
            "dims": (125.0, 50.0, 31.5),
            "price": 20.00,
            "type": "Desktop adapter",
        },
    }

    def __init__(self, size: str = "DR-120-24", **kwargs):
        if size not in self.psu_data:
            super().__init__(
                category="power_supply", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.psu_data[size]
        l, w, h = data["dims"]

        body = Box(l, w, h)
        body.color = Color("silver")

        super().__init__(
            category="power_supply", part_number=size, data=data, children=[body]
        )


class ElectronicRelay(COTSPart):
    """
    Represents a COTS Relay for switching.
    """

    relay_data = {
        "SRD-05VDC-SL-C": {
            "coil_voltage_v": 5.0,
            "contact_rating_a": 10.0,
            "weight_g": 10.0,
            "dims": (19.0, 15.5, 15.0),
            "price": 1.50,
        },
        "G5LE-1-DC24": {
            "coil_voltage_v": 24.0,
            "contact_rating_a": 10.0,
            "weight_g": 12.0,
            "dims": (22.5, 16.5, 19.0),
            "price": 3.00,
        },
    }

    def __init__(self, size: str = "G5LE-1-DC24", **kwargs):
        if size not in self.relay_data:
            super().__init__(
                category="relay", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.relay_data[size]
        l, w, h = data["dims"]

        body = Box(l, w, h)
        body.color = Color("blue")

        super().__init__(category="relay", part_number=size, data=data, children=[body])
