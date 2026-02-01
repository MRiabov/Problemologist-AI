import streamlit as st

from src.dashboard.components.chat import render_chat
from src.dashboard.components.code import render_code
from src.dashboard.components.sidebar import render_sidebar


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
            st.info("3D Visualization Component Placeholder (WP03)")
            # This is where the MuJoCo/Three.js view will go
            
    else:
        st.warning("Please select an episode from the sidebar or enable Live Mode.")

if __name__ == "__main__":
    main()
