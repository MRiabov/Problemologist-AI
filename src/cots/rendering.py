import os
import hashlib
import pyvista as pv
from build123d import export_stl, Part, Compound


from PIL import Image, ImageDraw, ImageFont


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
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir, exist_ok=True)

    # Generate a stable hash for the part
    part_hash = hashlib.md5(part_id.encode()).hexdigest()
    image_path = os.path.abspath(os.path.join(cache_dir, f"{part_hash}.png"))

    if os.path.exists(image_path):
        return image_path

    # Temporary STL path for rendering
    stl_path = os.path.join(cache_dir, f"{part_hash}.stl")

    try:
        # Export to STL first
        if isinstance(part_obj, (Part, Compound)):
            export_stl(part_obj, stl_path)
        else:
            raise ValueError(f"Unsupported part object type: {type(part_obj)}")

        # Use PyVista to render the STL to PNG
        mesh = pv.read(stl_path)

        # Setup plotter for offscreen rendering
        plotter = pv.Plotter(off_screen=True, window_size=[400, 400])
        plotter.set_background("white")
        plotter.add_mesh(mesh, color="lightblue", show_edges=True)
        plotter.camera_position = "iso"
        plotter.screenshot(image_path)
        plotter.close()

    except Exception as e:
        print(f"Warning: Failed to render part {part_id}: {e}")
        # Create a placeholder if rendering fails
        render_placeholder_image(image_path)
    finally:
        # Cleanup temporary STL
        if os.path.exists(stl_path):
            os.remove(stl_path)

    return image_path
