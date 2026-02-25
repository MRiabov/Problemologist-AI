from build123d import Box, Color

from shared.cots.base import COTSPart


class LogicBoard(COTSPart):
    """
    Represents a COTS Logic Board or Microcontroller.
    """

    board_data = {
        "Arduino-Uno": {
            "voltage_dc": 5.0,
            "max_current_a": 0.05,  # ~50mA idle
            "weight_g": 25.0,
            "dims": (68.6, 53.4, 15.0),
            "price": 20.00,
        },
        "Raspberry-Pi-4": {
            "voltage_dc": 5.0,
            "max_current_a": 0.6,   # ~600mA idle
            "weight_g": 46.0,
            "dims": (85.6, 56.5, 17.0),
            "price": 35.00,
        },
        "ESP32-DevKit": {
            "voltage_dc": 5.0,      # USB powered or Vin
            "max_current_a": 0.08,  # ~80mA
            "weight_g": 10.0,
            "dims": (52.0, 28.0, 13.0),
            "price": 6.00,
        },
        "Arduino-Nano": {
            "voltage_dc": 5.0,
            "max_current_a": 0.03,
            "weight_g": 7.0,
            "dims": (45.0, 18.0, 18.0), # with headers
            "price": 10.00,
        },
    }

    def __init__(self, size: str = "Arduino-Uno", **kwargs):
        if size not in self.board_data:
            super().__init__(
                category="logic_board", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.board_data[size]
        l, w, h = data["dims"]

        body = Box(l, w, h)
        body.color = Color("green") # PCBs are often green

        super().__init__(
            category="logic_board", part_number=size, data=data, children=[body]
        )
