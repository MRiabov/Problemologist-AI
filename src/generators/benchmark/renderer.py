import os
import mujoco
import numpy as np
from PIL import Image
from typing import List

def render_scenario(xml_string: str, output_path_prefix: str, width: int = 640, height: int = 480) -> List[str]:
    """
    Renders a MuJoCo scenario from multiple angles and saves the images.
    
    Args:
        xml_string: The MJCF XML string.
        output_path_prefix: Prefix for the output image files (e.g., 'path/to/image').
        width: Image width.
        height: Image height.
        
    Returns:
        List of paths to the saved images.
    """
    image_paths = []
    try:
        model = mujoco.MjModel.from_xml_string(xml_string)
        data = mujoco.MjData(model)
        
        # Ensure we have at least one step to settle initial positions
        mujoco.mj_forward(model, data)
        
        renderer = mujoco.Renderer(model, width=width, height=height)
        
        # 1. Render from defined cameras if any
        num_cameras = model.ncam
        if num_cameras > 0:
            for cam_id in range(num_cameras):
                renderer.update_scene(data, camera=cam_id)
                pixels = renderer.render()
                img = Image.fromarray(pixels)
                path = f"{output_path_prefix}_cam{cam_id}.png"
                img.save(path)
                image_paths.append(path)
        
        # 2. Always render from a few default angles to ensure coverage
        # We can use MjvCamera or just manipulate the scene
        # For simplicity with mujoco.Renderer, we use the default free camera if no cams exist,
        # or just add some standard views.
        
        if not image_paths:
            # Default free camera
            renderer.update_scene(data)
            pixels = renderer.render()
            img = Image.fromarray(pixels)
            path = f"{output_path_prefix}_default.png"
            img.save(path)
            image_paths.append(path)
            
        renderer.close()
        
    except Exception as e:
        print(f"Warning: Rendering failed: {e}")
        # Return empty list or handle as needed
        
    return image_paths
