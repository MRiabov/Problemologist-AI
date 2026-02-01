from pathlib import Path

import pyvista
import streamlit as st
from stpyvista import stpyvista


def render_3d_artifact(file_path: Path):
    """
    Renders an STL or mesh file using pyvista and stpyvista.
    """
    if not file_path.exists():
        st.error(f"File not found: {file_path}")
        return

    try:
        # Load mesh
        mesh = pyvista.read(str(file_path))
        
        # Setup Plotter
        plotter = pyvista.Plotter(window_size=[400, 400])
        plotter.add_mesh(mesh, color='white', lighting=True)
        plotter.add_axes()
        plotter.view_isometric()
        
        # Render
        stpyvista(plotter, key=f"3d_viewer_{file_path.name}")
        
    except Exception as e:
        st.error(f"Error rendering 3D artifact: {e!s}")
