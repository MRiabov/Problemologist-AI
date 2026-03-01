import json
import os
import sys
from pathlib import Path

from build123d import Compound, Part


def _emit_event(event_type: str, data: dict):
    """Emit an event for the simulation loop to pick up."""
    event = {
        "type": event_type,
        "data": data,
    }
    # Append to events.jsonl in the current directory
    with open("events.jsonl", "a") as f:
        f.write(json.dumps(event) + "\n")


def validate_and_price(component: Part | Compound):
    """
    Validates a part for manufacturability and prices it.
    This is a stub that emits an event for the system to process.
    """
    label = getattr(component, "label", "unnamed_component")
    print(f"Validating and pricing component: {label}")

    # In a real scenario, this would call the workbench logic.
    # For now, we emit an event that the worker can catch.
    _emit_event(
        "validate_and_price_requested",
        {
            "label": label,
            "is_compound": isinstance(component, Compound),
        },
    )


def simulate(component: Compound):
    """
    Submits a model for physics simulation.
    This is a stub that emits an event for the system to process.
    """
    label = getattr(component, "label", "assembly")
    print(f"Simulating assembly: {label}")

    _emit_event(
        "simulation_requested",
        {
            "label": label,
        },
    )


def preview_design(component: Part | Compound, pitch: float = -45.0, yaw: float = 45.0):
    """
    Triggers a CAD render of the component.
    """
    label = getattr(component, "label", "component")
    print(f"Previewing component: {label} (pitch={pitch}, yaw={yaw})")

    _emit_event(
        "preview_requested",
        {
            "label": label,
            "pitch": pitch,
            "yaw": yaw,
        },
    )


def submit_for_review(component: Compound):
    """
    Submits the design for review.
    """
    label = getattr(component, "label", "final_assembly")
    print(f"Submitting for review: {label}")

    _emit_event(
        "submit_for_review_requested",
        {
            "label": label,
        },
    )
