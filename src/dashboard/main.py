import streamlit as st
from streamlit_autorefresh import st_autorefresh

from src.dashboard.components.chat import render_chat
from src.dashboard.components.code import render_code
from src.dashboard.components.sidebar import render_sidebar
from src.dashboard.components.viewer_3d import render_3d_artifact
from src.dashboard.data import get_latest_episode, get_step_artifacts
from src.dashboard.utils import resolve_artifact_path


def init_session_state():
    """Initializes streamlit session state variables."""
    if "selected_episode_id" not in st.session_state:
        st.session_state.selected_episode_id = None
    if "selected_step_index" not in st.session_state:
        st.session_state.selected_step_index = 0
    if "live_mode" not in st.session_state:
        st.session_state.live_mode = False

def main():
    st.set_page_config(
        page_title="Problemologist Dashboard",
        layout="wide",
        initial_sidebar_state="expanded"
    )
    
    init_session_state()
    
    # Auto-refresh if in Live Mode
    if st.session_state.live_mode:
        st_autorefresh(interval=2000, key="live_refresh")
        # In live mode, force selection of latest
        latest = get_latest_episode()
        if latest:
            st.session_state.selected_episode_id = latest["id"]
            num_steps = len(latest.get("steps", []))
            st.session_state.selected_step_index = max(0, num_steps - 1)
    
    # Render Sidebar
    episode = render_sidebar()
    
    # Main Content Area
    st.title("Problemologist Agent Dashboard")
    
    if episode:
        col1, col2 = st.columns([1, 1])
        
        with col1:
            st.subheader("Agent Reasoning & Chat")
            # Filter steps up to selected index
            steps = episode.get("steps", [])
            render_chat(steps, st.session_state.selected_step_index)
            
            st.divider()
            st.subheader("Generated Code")
            if steps and st.session_state.selected_step_index < len(steps):
                render_code(steps[st.session_state.selected_step_index])
            else:
                st.info("No code available for this step.")
                
        with col2:
            st.subheader("3D Preview & Simulation")
            
            # Fetch artifacts for current step
            artifacts = get_step_artifacts(
                st.session_state.selected_episode_id, 
                st.session_state.selected_step_index
            )
            
            # Filter for 3D files
            three_d_files = [a for a in artifacts if a.endswith(('.stl', '.obj'))]
            
            if three_d_files:
                # If multiple, allow selection
                if len(three_d_files) > 1:
                    selected_file = st.selectbox("Select Mesh", three_d_files)
                else:
                    selected_file = three_d_files[0]
                
                path = resolve_artifact_path(selected_file)
                render_3d_artifact(path)
            else:
                st.info("No 3D artifacts generated in this step.")
            
    else:
        st.warning("Please select an episode from the sidebar or enable Live Mode.")

if __name__ == "__main__":
    main()