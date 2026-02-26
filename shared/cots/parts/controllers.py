from build123d import Box, Color
from shared.cots.base import COTSPart

class LogicBoard(COTSPart):
    """
    Represents a COTS Logic Board / Microcontroller.
    """
    board_data = {
        "Arduino-Uno": {
            "price": 22.00,
            "weight_g": 25.0,
            "dims": (68.6, 53.4, 15.0),
            "current_draw_a": 0.05,
        }
    }

    def __init__(self, size: str = "Arduino-Uno", **kwargs):
        if size not in self.board_data:
            super().__init__(category="logic_board", part_number="UNKNOWN", data={}, children=[])
            return

        data = self.board_data[size]
        l, w, h = data["dims"]
        body = Box(l, w, h)
        body.color = Color("green")
        super().__init__(category="logic_board", part_number=size, data=data, children=[body])
