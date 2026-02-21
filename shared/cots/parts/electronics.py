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
        "6V-Battery": {
            "voltage_dc": 6.0,
            "max_current_a": 4.0,
            "weight_g": 150.0,
            "dims": (50.0, 30.0, 20.0),
            "price": 5.00,
        },
        "12V-Battery": {
            "voltage_dc": 12.0,
            "max_current_a": 7.0,
            "weight_g": 450.0,
            "dims": (151.0, 65.0, 94.0),
            "price": 20.00,
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


class Switch(COTSPart):
    """
    Represents a COTS Electrical Switch.
    """

    switch_data = {
        "Rocker-Switch": {
            "voltage_max": 250.0,
            "current_max": 6.0,
            "weight_g": 5.0,
            "dims": (21.0, 15.0, 25.0),
            "price": 0.50,
        },
        "Toggle-Switch": {
            "voltage_max": 250.0,
            "current_max": 15.0,
            "weight_g": 20.0,
            "dims": (30.0, 15.0, 50.0),
            "price": 2.00,
        },
    }

    def __init__(self, size: str = "Rocker-Switch", **kwargs):
        if size not in self.switch_data:
            super().__init__(
                category="switch", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.switch_data[size]
        l, w, h = data["dims"]
        body = Box(l, w, h)
        body.color = Color("red")
        super().__init__(
            category="switch", part_number=size, data=data, children=[body]
        )


class Connector(COTSPart):
    """
    Represents a COTS Electrical Connector.
    """

    connector_data = {
        "XT60": {
            "max_current_a": 60.0,
            "weight_g": 7.0,
            "dims": (15.0, 16.0, 8.0),
            "price": 0.80,
        },
        "JST-XH-2P": {
            "max_current_a": 3.0,
            "weight_g": 1.0,
            "dims": (10.0, 6.0, 6.0),
            "price": 0.10,
        },
    }

    def __init__(self, size: str = "XT60", **kwargs):
        if size not in self.connector_data:
            super().__init__(
                category="connector", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.connector_data[size]
        l, w, h = data["dims"]
        body = Box(l, w, h)
        body.color = Color("yellow")
        super().__init__(
            category="connector", part_number=size, data=data, children=[body]
        )


class Wire(COTSPart):
    """
    Represents a COTS Electrical Wire.
    """

    wire_data = {
        "AWG18-RED": {
            "gauge_awg": 18,
            "max_current_a": 16.0,
            "weight_g_per_m": 20.0,
            "price_per_m": 0.50,
            "color": "red",
        },
        "AWG24-BLACK": {
            "gauge_awg": 24,
            "max_current_a": 3.5,
            "weight_g_per_m": 5.0,
            "price_per_m": 0.20,
            "color": "black",
        },
    }

    def __init__(self, size: str = "AWG18-RED", **kwargs):
        if size not in self.wire_data:
            super().__init__(
                category="wire", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.wire_data[size]
        # Wire doesn't have fixed dims in the same way, but we can give it a small spool or representation
        body = Box(10.0, 10.0, 2.0)
        body.color = Color(data["color"])

        # Cost and weight for COTS items in DB are usually PER UNIT.
        # For wire, let's treat it as 1m unit for indexing.
        data["weight_g"] = data["weight_g_per_m"]
        data["price"] = data["price_per_m"]

        super().__init__(category="wire", part_number=size, data=data, children=[body])
