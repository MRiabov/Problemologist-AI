from typing import Any, Union
from build123d import Part, Compound, Location
import structlog

logger = structlog.get_logger(__name__)

def validate_and_price(component: Union[Part, Compound]) -> dict[str, Any]:
    """
    Validates a build123d component and calculates its estimated manufacturing cost.
    
    Args:
        component: A build123d Part or Compound object.
        
    Returns:
        A dictionary containing validation status and cost.
    """
    logger.info("validate_and_price", type=type(component))
    
    if not isinstance(component, (Part, Compound)):
        raise TypeError(f"Expected build123d Part or Compound, got {type(component)}")

    # Calculate bounding box for "complexity"
    bbox = component.bounding_box()
    volume = component.volume
    
    # Simple mock pricing logic
    # Base cost + volume-based cost
    base_cost = 10.0
    volume_multiplier = 0.001
    cost = base_cost + (volume * volume_multiplier)
    
    return {
        "valid": True,
        "cost": round(cost, 2),
        "dimensions": {
            "min": [bbox.min.X, bbox.min.Y, bbox.min.Z],
            "max": [bbox.max.X, bbox.max.Y, bbox.max.Z],
        },
        "volume": round(volume, 2)
    }
