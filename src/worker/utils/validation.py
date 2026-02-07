from typing import Any, Union
from build123d import Part, Compound, Location
import structlog

logger = structlog.get_logger(__name__)

# validate_and_price moved to dfm.py