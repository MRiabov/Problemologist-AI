import hashlib
from pathlib import Path

import pyvista as pv
from build123d import Compound, Part, export_stl
from PIL import Image, ImageDraw

from src.agent.utils.logging import get_logger

logger = get_logger(__name__)


def render_placeholder_image(output_path: str):
    """Generate a simple 'Preview Not Available' placeholder image."""
    img = Image.new("RGB", (400, 400), color=(240, 240, 240))
    d = ImageDraw.Draw(img)
    # Draw a border
    d.rectangle([10, 10, 390, 390], outline=(200, 200, 200), width=2)
    # Draw text (centered)
    text = "Preview Not\nAvailable"
    d.text((200, 200), text, fill=(100, 100, 100), anchor="mm", align="center")
    img.save(output_path)


def render_part(part_obj, part_id: str, cache_dir: str = ".cache/cots") -> str:
    """
    Render a build123d object to a PNG file.
    Returns the absolute path to the rendered image.
    Uses hash-based caching to avoid redundant renders.
    """
    cache_path = Path(cache_dir)
    if not cache_path.exists():
        cache_path.mkdir(parents=True, exist_ok=True)

    # Generate a stable hash for the part
    part_hash = hashlib.md5(part_id.encode()).hexdigest()
    image_path = (cache_path / f"{part_hash}.png").resolve()

    if image_path.exists():
        return str(image_path)

    # Temporary STL path for rendering
    stl_path = cache_path / f"{part_hash}.stl"

    try:
        # Export to STL first
        if isinstance(part_obj, (Part, Compound)):
            export_stl(part_obj, str(stl_path))
        else:
            raise ValueError(f"Unsupported part object type: {type(part_obj)}")

        # Use PyVista to render the STL to PNG
        mesh = pv.read(str(stl_path))

        # Setup plotter for offscreen rendering
        plotter = pv.Plotter(off_screen=True, window_size=[400, 400])
        plotter.set_background("white")
        plotter.add_mesh(mesh, color="lightblue", show_edges=True)
        plotter.camera_position = "iso"
        plotter.screenshot(str(image_path))
        plotter.close()

    except Exception as e:
        logger.warning("Failed to render part", part_id=part_id, error=str(e))
        # Create a placeholder if rendering fails
        render_placeholder_image(str(image_path))
    finally:
        # Cleanup temporary STL
        if stl_path.exists():
            stl_path.unlink()

    return str(image_path)
