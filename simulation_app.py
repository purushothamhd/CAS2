# simulation_app.py - WITH DRONE SPEED CONTROL

import streamlit as st
import json
import os
import multiprocessing as mp
import queue
import socket
import time
from physics_engine import simulation_process
from visualizer import visualization_process

def is_port_in_use(port):
    """Check if a port is already in use"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(('localhost', port))
            return False
        except OSError:
            return True

def kill_port(port):
    """Try to kill the process using the port"""
    import subprocess
    try:
        subprocess.run(f"lsof -ti:{port} | xargs kill -9", 
                      shell=True, stderr=subprocess.DEVNULL)
        time.sleep(1)
        return True
    except:
        return False

st.set_page_config(layout="wide")
st.title("Physics-Based Homeostasis Simulation")
st.markdown("A real-time N-body simulation where homeostasis emerges from physical laws.")

# Default parameters with new speed controls
defaults = {
    's0_epsilon': 2.0,
    's1_epsilon': 5.0,
    'energy_threshold_s1': 150.0,
    'drone_mode': 'Idle',
    'drone_orbit_speed': 0.02,    # NEW: Default orbit speed
    'drone_patrol_speed': 0.01    # NEW: Default patrol speed
}

# Initialize state
if 'sim_running' not in st.session_state:
    st.session_state.sim_running = False
    st.session_state.s0_epsilon = defaults['s0_epsilon']
    st.session_state.s1_epsilon = defaults['s1_epsilon']
    st.session_state.energy_threshold = defaults['energy_threshold_s1']
    st.session_state.drone_mode = defaults['drone_mode']
    st.session_state.drone_orbit_speed = defaults['drone_orbit_speed']      # NEW
    st.session_state.drone_patrol_speed = defaults['drone_patrol_speed']    # NEW
    
    # Process and queue handles
    st.session_state.data_q = None
    st.session_state.param_q = None
    st.session_state.sim_process = None
    st.session_state.vis_process = None

# Start/Stop Buttons
col1, col2, col3 = st.columns(3)
with col1:
    if st.button("🚀 Start Simulation", use_container_width=True, disabled=st.session_state.sim_running):
        # Check if port 5006 is available
        if is_port_in_use(5006):
            st.warning("Port 5006 is in use. Attempting to free it...")
            if kill_port(5006):
                st.success("Port freed successfully!")
                time.sleep(0.5)  # Give OS time to fully release
            else:
                st.error("Could not free port 5006. Please manually stop the previous process.")
                st.stop()
        
        st.session_state.data_q = mp.Queue()
        st.session_state.param_q = mp.Queue()
        
        st.session_state.sim_process = mp.Process(
            target=simulation_process, 
            args=(st.session_state.data_q, st.session_state.param_q)
        )
        st.session_state.vis_process = mp.Process(
            target=visualization_process, 
            args=(st.session_state.data_q,)
        )
        
        st.session_state.sim_process.start()
        st.session_state.vis_process.start()
        
        # Send START command with initial parameters
        st.session_state.param_q.put({'command': 'START'})
        st.session_state.sim_running = True
        st.rerun()

with col2:
    if st.button("🛑 Stop Simulation", use_container_width=True, disabled=not st.session_state.sim_running):
        # Send STOP command
        if st.session_state.param_q:
            try:
                st.session_state.param_q.put({'command': 'STOP'})
            except:
                pass
        
        # Terminate processes forcefully (don't wait)
        if st.session_state.vis_process and st.session_state.vis_process.is_alive():
            st.session_state.vis_process.terminate()
            st.session_state.vis_process.join(timeout=0.5)
            if st.session_state.vis_process.is_alive():
                st.session_state.vis_process.kill()
        
        if st.session_state.sim_process and st.session_state.sim_process.is_alive():
            st.session_state.sim_process.terminate()
            st.session_state.sim_process.join(timeout=0.5)
            if st.session_state.sim_process.is_alive():
                st.session_state.sim_process.kill()
        
        # Clear references
        st.session_state.sim_running = False
        st.session_state.data_q = None
        st.session_state.param_q = None
        st.session_state.sim_process = None
        st.session_state.vis_process = None
        
        # Give the OS time to release the port
        import time
        time.sleep(1)
        
        st.rerun()

with col3:
    if st.button("📸 Log Snapshot", use_container_width=True, disabled=not st.session_state.sim_running, type="secondary"):
        if st.session_state.param_q and st.session_state.sim_running:
            try:
                st.session_state.param_q.put({'command': 'LOG_SNAPSHOT'})
                st.success("✅ Snapshot command sent! Check console for filename.", icon="📸")
                st.balloons()
            except Exception as e:
                st.error(f"Failed to send snapshot command: {e}")

if st.session_state.sim_running:
    st.success("✅ Simulation is running. View the visualization at http://localhost:5006")
    st.info("💡 Click '📸 Log Snapshot' to save current particle states to CSV")
else:
    st.info("Simulation is stopped.")

st.divider()

st.header("🔬 Real-Time Parameter Tuning")
st.write("Adjust the values below, then click 'Apply Changes' to update the simulation.")

# Function to send params over the queue
def update_params():
    params = {
        's0_epsilon': st.session_state.s0_epsilon,
        's1_epsilon': st.session_state.s1_epsilon,
        'energy_threshold_s1': st.session_state.energy_threshold,
        'drone_mode': st.session_state.drone_mode.lower(),
        'drone_orbit_speed': st.session_state.drone_orbit_speed,      # NEW
        'drone_patrol_speed': st.session_state.drone_patrol_speed     # NEW
    }
    
    if st.session_state.param_q and st.session_state.sim_running:
        try:
            st.session_state.param_q.put({'params': params})
            st.toast("Parameters applied to simulation!", icon="🔬")
        except Exception as e:
            st.error(f"Failed to send parameters: {e}")
    elif not st.session_state.sim_running:
        st.info("Simulation is stopped. Start the simulation to apply parameters.")

# Drone Control Section
st.subheader("🚁 Drone Agent Control")
st.selectbox("Drone Behavior", ('Idle', 'Patrol', 'Orbit'), key='drone_mode')

# NEW: Speed controls - only show relevant slider based on mode
speed_col1, speed_col2 = st.columns(2)
with speed_col1:
    if st.session_state.drone_mode == 'Orbit':
        st.slider(
            "🔄 Orbit Speed (Angular Velocity)", 
            min_value=0.005, 
            max_value=0.1, 
            value=st.session_state.drone_orbit_speed,
            step=0.005,
            key='drone_orbit_speed',
            help="Controls how fast the drone orbits around the center"
        )
    else:
        st.markdown("*Orbit speed control available in Orbit mode*")

with speed_col2:
    if st.session_state.drone_mode == 'Patrol':
        st.slider(
            "↔️ Patrol Speed (Frequency)", 
            min_value=0.002, 
            max_value=0.05, 
            value=st.session_state.drone_patrol_speed,
            step=0.002,
            key='drone_patrol_speed',
            help="Controls how fast the drone patrols horizontally"
        )
    else:
        st.markdown("*Patrol speed control available in Patrol mode*")

st.divider()

# Particle Interaction Control
st.subheader("⚛️ Particle Interaction Control")
c1, c2, c3 = st.columns(3)
with c1:
    st.slider("S0 Attraction (Epsilon)", 0.5, 10.0, key='s0_epsilon', step=0.1)
with c2:
    st.slider("S1 Attraction (Epsilon)", 1.0, 20.0, key='s1_epsilon', step=0.1)
with c3:
    st.slider("S1 Energy Threshold", 50.0, 500.0, key='energy_threshold', step=5.0)

st.divider()

# Apply Changes Button
if st.button("✅ Apply Changes", use_container_width=True, type="primary"):
    update_params()

st.divider()

# Live Status Display
status_col1, status_col2, status_col3 = st.columns(3)
with status_col1:
    st.metric(label="Drone Mode", value=st.session_state.drone_mode)
with status_col2:
    if st.session_state.drone_mode == 'Orbit':
        st.metric(label="Orbit Speed", value=f"{st.session_state.drone_orbit_speed:.3f}")
    elif st.session_state.drone_mode == 'Patrol':
        st.metric(label="Patrol Speed", value=f"{st.session_state.drone_patrol_speed:.3f}")
    else:
        st.metric(label="Speed", value="N/A")
with status_col3:
    st.metric(label="Simulation", value="Running" if st.session_state.sim_running else "Stopped")

st.divider()

# Snapshot Information
with st.expander("📋 Snapshot Data Information"):
    st.markdown("""
    ### What gets logged in the CSV snapshot:
    
    **Per Particle Data (100 rows):**
    - **Identification**: `particle_id`, `is_drone`, `state` (S0/S1)
    - **Position**: `position_x`, `position_y`
    - **Velocity**: `velocity_x`, `velocity_y`, `velocity_magnitude`, `speed`
    - **Acceleration**: `acceleration_x`, `acceleration_y`, `acceleration_magnitude`
    - **Forces**: `force_x`, `force_y`, `force_magnitude`
    - **Properties**: `mass`, `sigma`, `epsilon`
    - **Energy**: `kinetic_energy`, `potential_energy`, `total_energy`
    - **Metadata**: `tick` (simulation frame number)
    
    **File naming**: `simulation_snapshot_tick_<tick>_<timestamp>.csv`
    
    **Location**: Saved in the same directory as the application.
    
    ⚠️ **Note**: Snapshots capture the exact state at the moment the button is clicked.
    """)

