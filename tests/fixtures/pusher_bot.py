from build123d import Part, Box, export_stl
import os


def create_pusher_geometry(filepath: str):
    """
    Creates a simple cube pusher geometry.
    """
    pusher = Box(0.1, 0.1, 0.1)
    export_stl(pusher, filepath)
    return filepath


PUSHER_SCRIPT = """
import mujoco
import numpy as np

def control_logic(model, data):
    # Apply constant forward force to the actuator or directly to qfrc
    # In this simple case, we'll just push the body forward
    data.qfrc_applied[0] = 10.0  # Simple forward push
"""
