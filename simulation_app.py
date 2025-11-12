# simulation_app.py

import streamlit as st
import json
import os
import multiprocessing as mp
import queue  # Use the standard queue for exception handling
from physics_engine import simulation_process
from visualizer import visualization_process

st.set_page_config(layout="wide")
st.title("Physics-Based Homeostasis Simulation")
st.markdown("A real-time N-body simulation where homeostasis emerges from physical laws.")

# --- NEW: Default parameters (no more file loading) ---
defaults = {
    's0_epsilon': 2.0,
    's1_epsilon': 5.0,
    'energy_threshold_s1': 150.0,
    'drone_mode': 'Idle' # Use capitalized version for the selectbox
}

# --- MODIFIED: Initialize state ---
if 'sim_running' not in st.session_state:
    st.session_state.sim_running = False
    st.session_state.s0_epsilon = defaults['s0_epsilon']
    st.session_state.s1_epsilon = defaults['s1_epsilon']
    st.session_state.energy_threshold = defaults['energy_threshold_s1']
    st.session_state.drone_mode = defaults['drone_mode']
    
    # These will hold our processes and queues
    st.session_state.data_q = None
    st.session_state.param_q = None # NEW queue for UI -> Engine
    st.session_state.sim_process = None
    st.session_state.vis_process = None

# --- UI Layout (Buttons are MODIFIED) ---
col1, col2 = st.columns(2)
with col1:
    if st.button("🚀 Start Simulation", use_container_width=True, disabled=st.session_state.sim_running):
        # --- NEW: Start processes ---
        st.session_state.data_q = mp.Queue()
        st.session_state.param_q = mp.Queue()
        
        st.session_state.sim_process = mp.Process(
            target=simulation_process, 
            args=(st.session_state.data_q, st.session_state.param_q) # Pass param_q
        )
        st.session_state.vis_process = mp.Process(
            target=visualization_process, 
            args=(st.session_state.data_q,)
        )
        
        st.session_state.sim_process.start()
        st.session_state.vis_process.start()
        
        # Send the "START" command
        st.session_state.param_q.put({'command': 'START'})
        st.session_state.sim_running = True
        st.rerun()

with col2:
    if st.button("🛑 Stop Simulation", use_container_width=True, disabled=not st.session_state.sim_running):
        # --- NEW: Stop and terminate processes ---
        if st.session_state.param_q:
            st.session_state.param_q.put({'command': 'STOP'})
        
        # Give processes a moment to stop gracefully before terminating
        if st.session_state.sim_process:
            st.session_state.sim_process.join(timeout=1)
            st.session_state.sim_process.terminate()
        if st.session_state.vis_process:
            st.session_state.vis_process.join(timeout=1)
            st.session_state.vis_process.terminate()
        
        st.session_state.sim_running = False
        st.session_state.data_q = None
        st.session_state.param_q = None
        st.session_state.sim_process = None
        st.session_state.vis_process = None
        st.rerun()

if st.session_state.sim_running:
    st.success("✅ Simulation is running. View the visualization at http://localhost:5006")
else:
    st.info("Simulation is stopped.")

st.divider()

st.header("🔬 Real-Time Parameter Tuning")
st.write("Adjust the values below, then click 'Apply Changes' to update the simulation.")

# --- MODIFIED: The function to send params over the queue ---
def update_params():
    params = {
        's0_epsilon': st.session_state.s0_epsilon,
        's1_epsilon': st.session_state.s1_epsilon,
        'energy_threshold_s1': st.session_state.energy_threshold,
        'drone_mode': st.session_state.drone_mode.lower() # Send lowercase version
    }
    
    # --- NEW: Send params over the queue ---
    if st.session_state.param_q and st.session_state.sim_running:
        try:
            st.session_state.param_q.put({'params': params})
            st.toast("Parameters applied to simulation!", icon="🔬")
        except Exception as e:
            st.error(f"Failed to send parameters: {e}")
    elif not st.session_state.sim_running:
        st.info("Simulation is stopped. Start the simulation to apply parameters.")

# (The rest of the UI elements are unchanged)
st.subheader("🚁 Drone Agent Control")
st.selectbox("Drone Behavior", ('Idle', 'Patrol', 'Orbit'), key='drone_mode')

st.subheader(" Particle Interaction Control")
c1, c2, c3 = st.columns(3)
with c1:
    st.slider("S0 Attraction (Epsilon)", 0.5, 10.0, key='s0_epsilon', step=0.1)
with c2:
    st.slider("S1 Attraction (Epsilon)", 1.0, 20.0, key='s1_epsilon', step=0.1)
with c3:
    st.slider("S1 Energy Threshold", 50.0, 500.0, key='energy_threshold', step=5.0)

st.divider()

if st.button("✅ Apply Changes", use_container_width=True, type="primary"):
    update_params()

st.divider()

# --- Live Status Display ---
status_col, _ = st.columns([1, 3])
with status_col:
    st.metric(label="Drone Status", value=st.session_state.drone_mode)
