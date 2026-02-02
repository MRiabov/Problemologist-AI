import streamlit as st

from src.dashboard.data import get_all_episodes, get_episode_by_id, insert_step


def render_sidebar():
    """Renders the dashboard sidebar for navigation."""
    st.sidebar.header("Dashboard Controls")
    
    # Mode Selector
    mode = st.sidebar.radio(
        "Dashboard Mode", 
        ["Viewer", "Benchmark Generator"],
        key="dashboard_mode_selector"
    )
    st.session_state.app_mode = mode
    
    if mode == "Benchmark Generator":
        return None

    # Mode Toggle
    st.session_state.live_mode = st.sidebar.checkbox(
        "Live Mode", 
        value=st.session_state.live_mode
    )
    
    if st.session_state.live_mode:
        st.sidebar.success("🔴 Streaming live from agent...")
        return None # In a real impl, this would return live state
    
    st.sidebar.divider()
    st.sidebar.subheader("Episode History")
    
    episodes = get_all_episodes()
    episode_options = {
        f"[{ep['timestamp'].strftime('%H:%M')}] {ep['name']}": ep['id'] 
        for ep in episodes
    }
    
    if st.session_state.selected_episode_id is None:
        default_index = 0
    else:
        try:
            default_index = list(episode_options.values()).index(
                st.session_state.selected_episode_id
            )
        except ValueError:
            default_index = 0

    selected_label = st.sidebar.selectbox(
        "Select Episode",
        options=list(episode_options.keys()),
        index=default_index
    )
    
    if selected_label:
        episode_id = episode_options[selected_label]
        
        # Reset step index if we switched episodes
        if episode_id != st.session_state.selected_episode_id:
            st.session_state.selected_episode_id = episode_id
            st.session_state.selected_step_index = 0
        
        # Fetch full episode data
        episode = get_episode_by_id(episode_id)
        steps = episode.get("steps", [])
        
        if steps:
            st.sidebar.subheader("Step Navigation")
            step_index = st.sidebar.slider(
                "Navigate Steps",
                min_value=0,
                max_value=len(steps) - 1,
                value=st.session_state.selected_step_index
            )
            st.session_state.selected_step_index = step_index
            
        st.sidebar.divider()
        with st.sidebar.expander("🛠️ Debug: Insert Own Plan"):
            plan_input = st.text_area("Implementation Plan", height=150)
            if st.button("Insert Plan"):
                if plan_input:
                    insert_step(episode_id, "thought", plan_input)
                    st.success("Plan inserted!")
                    st.rerun()
                else:
                    st.error("Please enter a plan.")

        return episode
        
    return None
