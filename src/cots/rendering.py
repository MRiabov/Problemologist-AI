import os
import hashlib
import pyvista as pv
from build123d import export_stl, Part, Compound


def render_part(part_obj, part_id: str, cache_dir: str = ".cache/cots") -> str:
    """
    Render a build123d object to a PNG file.
    Returns the absolute path to the rendered image.
    Uses hash-based caching to avoid redundant renders.
    """
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir, exist_ok=True)

    # Generate a stable hash for the part
    # Since part_obj might be complex, we use the part_id as the primary key
    # but ideally we'd hash the geometry if part_id isn't unique enough.
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
            # Fallback if it's not a build123d object we recognize
            # but usually it should be.
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
        # Create a tiny placeholder if rendering fails
        # This prevents the whole pipeline from crashing
        return ""
    finally:
        # Cleanup temporary STL
        if os.path.exists(stl_path):
            os.remove(stl_path)

    return image_path
