from build123d import Box, Color

from shared.cots.base import COTSPart


class LogicBoard(COTSPart):
    """
    Represents a COTS Logic Board / Microcontroller.
    """

    board_data = {
        "Arduino-Uno-R3": {
            "voltage_input_min": 7.0,
            "voltage_input_max": 12.0,
            "current_draw_a": 0.05,
            "weight_g": 25.0,
            "dims": (68.6, 53.4, 15.0),
            "price": 23.00,
            "gpio_count": 14,
            "adc_count": 6,
        },
        "Raspberry-Pi-4B": {
            "voltage_input_min": 4.8,
            "voltage_input_max": 5.2,
            "current_draw_a": 0.6,
            "weight_g": 46.0,
            "dims": (85.0, 56.0, 17.0),
            "price": 35.00,
            "gpio_count": 40,
        },
        "ESP32-DevKitC": {
            "voltage_input_min": 4.5,
            "voltage_input_max": 9.0,  # On 5V pin
            "current_draw_a": 0.08,
            "weight_g": 10.0,
            "dims": (55.0, 28.0, 13.0),
            "price": 6.00,
            "gpio_count": 38,
        },
    }

    def __init__(self, size: str = "Arduino-Uno-R3", **kwargs):
        if size not in self.board_data:
            super().__init__(
                category="logic_board", part_number="UNKNOWN", data={}, children=[]
            )
            return

        data = self.board_data[size]
        l, w, h = data["dims"]
        body = Box(l, w, h)
        body.color = Color("teal")  # PCB color

        super().__init__(
            category="logic_board", part_number=size, data=data, children=[body]
        )
