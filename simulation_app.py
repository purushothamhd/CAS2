# simulation_app.py - FORCE IMPULSE CONTROL INTERFACE

import streamlit as st
import multiprocessing as mp
import socket
import subprocess
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
    try:
        subprocess.run(
            f"lsof -ti:{port} | xargs kill -9",
            shell=True,
            stderr=subprocess.DEVNULL
        )
        time.sleep(1)
        return True
    except:
        return False

# Page configuration
st.set_page_config(layout="wide", page_title="Drone Physics Control")
st.title("🚁 Physics-Based Drone Control System")
st.markdown("Control a drone using **pure force impulses** in a physics-simulated particle environment.")

# Default parameters
DEFAULTS = {
    's0_epsilon': 2.0,
    's1_epsilon': 5.0,
    'energy_threshold_s1': 150.0,
    'impulse_magnitude': 5000.0,
    'force_x': 1.0,
    'force_y': 0.0
}

# Initialize session state
def init_session_state():
    """Initialize all session state variables"""
    state_vars = {
        'sim_running': False,
        's0_epsilon': DEFAULTS['s0_epsilon'],
        's1_epsilon': DEFAULTS['s1_epsilon'],
        'energy_threshold': DEFAULTS['energy_threshold_s1'],
        'impulse_magnitude': DEFAULTS['impulse_magnitude'],
        'force_x': DEFAULTS['force_x'],
        'force_y': DEFAULTS['force_y'],
        'impulse_count': 0,
        'data_q': None,
        'param_q': None,
        'sim_process': None,
        'vis_process': None
    }
    
    for key, default_value in state_vars.items():
        if key not in st.session_state:
            st.session_state[key] = default_value

init_session_state()

# ===================== CONTROL FUNCTIONS =====================

def start_simulation():
    """Start the simulation processes"""
    # Check and free port if necessary
    if is_port_in_use(5006):
        st.warning("Port 5006 is in use. Attempting to free it...")
        if kill_port(5006):
            st.success("Port freed successfully!")
            time.sleep(0.5)
        else:
            st.error("Could not free port 5006. Please manually stop the previous process.")
            return False
    
    # Create queues
    st.session_state.data_q = mp.Queue()
    st.session_state.param_q = mp.Queue()
    
    # Create and start processes
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
    
    # Send start command
    st.session_state.param_q.put({'command': 'START'})
    st.session_state.sim_running = True
    st.session_state.impulse_count = 0
    
    return True

def stop_simulation():
    """Stop the simulation processes"""
    # Send stop command
    if st.session_state.param_q:
        try:
            st.session_state.param_q.put({'command': 'STOP'})
        except:
            pass
    
    # Terminate processes
    for process in [st.session_state.vis_process, st.session_state.sim_process]:
        if process and process.is_alive():
            process.terminate()
            process.join(timeout=0.5)
            if process.is_alive():
                process.kill()
    
    # Reset state
    st.session_state.sim_running = False
    st.session_state.data_q = None
    st.session_state.param_q = None
    st.session_state.sim_process = None
    st.session_state.vis_process = None

def apply_impulse():
    """Apply force impulse to drone"""
    if not st.session_state.param_q or not st.session_state.sim_running:
        st.warning("Simulation must be running to apply impulses")
        return False
    
    try:
        # Send impulse command
        st.session_state.param_q.put({
            'command': 'APPLY_IMPULSE',
            'force_x': st.session_state.force_x,
            'force_y': st.session_state.force_y
        })
        
        # Update magnitude parameter
        st.session_state.param_q.put({
            'params': {'impulse_magnitude': st.session_state.impulse_magnitude}
        })
        
        st.session_state.impulse_count += 1
        return True
    except Exception as e:
        st.error(f"Failed to apply impulse: {e}")
        return False

def update_physics_params():
    """Update physics parameters"""
    if not st.session_state.param_q or not st.session_state.sim_running:
        st.info("Simulation is stopped. Start the simulation to apply parameters.")
        return False
    
    params = {
        's0_epsilon': st.session_state.s0_epsilon,
        's1_epsilon': st.session_state.s1_epsilon,
        'energy_threshold_s1': st.session_state.energy_threshold,
        'impulse_magnitude': st.session_state.impulse_magnitude
    }
    
    try:
        st.session_state.param_q.put({'params': params})
        return True
    except Exception as e:
        st.error(f"Failed to send parameters: {e}")
        return False

# ===================== MAIN CONTROL PANEL =====================
st.header("🎮 Main Control Panel")

col1, col2, col3 = st.columns(3)

with col1:
    if st.button(
        "🚀 Start Simulation",
        use_container_width=True,
        disabled=st.session_state.sim_running
    ):
        if start_simulation():
            st.rerun()

with col2:
    if st.button(
        "🛑 Stop Simulation",
        use_container_width=True,
        disabled=not st.session_state.sim_running
    ):
        stop_simulation()
        time.sleep(1)
        st.rerun()

with col3:
    if st.button(
        "📸 Log Snapshot",
        use_container_width=True,
        disabled=not st.session_state.sim_running,
        type="secondary"
    ):
        if st.session_state.param_q and st.session_state.sim_running:
            try:
                st.session_state.param_q.put({'command': 'LOG_SNAPSHOT'})
                st.success("✅ Snapshot command sent!", icon="📸")
                st.balloons()
            except Exception as e:
                st.error(f"Failed to send snapshot command: {e}")

# Status Display
if st.session_state.sim_running:
    st.success("✅ Simulation is running. View at http://localhost:5006")
else:
    st.info("⏸️ Simulation is stopped. Click 'Start Simulation' to begin.")

st.divider()

# ===================== DRONE FORCE CONTROL =====================
st.header("🚁 Drone Force Impulse Control")

st.markdown("""
**How it works:**
- Specify the **direction** of force using X and Y components
- Adjust the **magnitude** (strength) of the impulse
- Click **"Apply Force Impulse"** to push the drone
- Each click adds momentum according to F = ma
""")

# Force Direction Input
force_col1, force_col2 = st.columns(2)

with force_col1:
    st.number_input(
        "Force Direction X",
        min_value=-10.0,
        max_value=10.0,
        value=st.session_state.force_x,
        step=0.1,
        key='force_x',
        help="Horizontal component (right is positive)"
    )

with force_col2:
    st.number_input(
        "Force Direction Y",
        min_value=-10.0,
        max_value=10.0,
        value=st.session_state.force_y,
        step=0.1,
        key='force_y',
        help="Vertical component (down is positive)"
    )

# Force Magnitude Slider
st.slider(
    "⚡ Impulse Magnitude (Force Strength)",
    min_value=1000.0,
    max_value=20000.0,
    value=st.session_state.impulse_magnitude,
    step=500.0,
    key='impulse_magnitude',
    help="Strength of the force impulse"
)

# Direction Indicator
direction_magnitude = (st.session_state.force_x**2 + st.session_state.force_y**2)**0.5
if direction_magnitude > 0:
    norm_x = st.session_state.force_x / direction_magnitude
    norm_y = st.session_state.force_y / direction_magnitude
    st.info(f"🎯 Normalized Direction: ({norm_x:.3f}, {norm_y:.3f}) | Magnitude: {st.session_state.impulse_magnitude:.0f}N")
else:
    st.warning("⚠️ Zero force vector - please set X or Y to a non-zero value")

# Apply Force Button
impulse_col1, impulse_col2 = st.columns([3, 1])

with impulse_col1:
    if st.button(
        "⚡ APPLY FORCE IMPULSE TO DRONE",
        use_container_width=True,
        type="primary",
        disabled=not st.session_state.sim_running
    ):
        if apply_impulse():
            st.success(f"🚀 Force impulse #{st.session_state.impulse_count} applied!", icon="⚡")

with impulse_col2:
    st.metric("Impulses Applied", st.session_state.impulse_count)

st.divider()

# ===================== QUICK PRESETS =====================
st.header("🎯 Quick Force Presets")
preset_cols = st.columns(5)

presets = [
    ("➡️ Right", (1.0, 0.0)),
    ("⬅️ Left", (-1.0, 0.0)),
    ("⬇️ Down", (0.0, 1.0)),
    ("⬆️ Up", (0.0, -1.0)),
    ("🔄 Reset", (0.0, 0.0))
]

for col, (label, (x, y)) in zip(preset_cols, presets):
    with col:
        if st.button(label, use_container_width=True):
            st.session_state.force_x = x
            st.session_state.force_y = y
            st.rerun()

st.divider()

# ===================== PHYSICS PARAMETERS =====================
st.header("⚙️ Physics Parameters")

st.subheader("⚛️ Particle Interaction Control")
p_col1, p_col2, p_col3 = st.columns(3)

with p_col1:
    st.slider(
        "S0 Attraction (Epsilon)",
        min_value=0.5,
        max_value=10.0,
        value=st.session_state.s0_epsilon,
        step=0.1,
        key='s0_epsilon',
        help="Attraction strength for calm (S0) particles"
    )

with p_col2:
    st.slider(
        "S1 Attraction (Epsilon)",
        min_value=1.0,
        max_value=20.0,
        value=st.session_state.s1_epsilon,
        step=0.1,
        key='s1_epsilon',
        help="Attraction strength for excited (S1) particles"
    )

with p_col3:
    st.slider(
        "S1 Energy Threshold",
        min_value=50.0,
        max_value=500.0,
        value=st.session_state.energy_threshold,
        step=5.0,
        key='energy_threshold',
        help="Energy required for S0→S1 transition"
    )

# Apply Parameters Button
if st.button("✅ Apply Physics Parameters", use_container_width=True, type="secondary"):
    if update_physics_params():
        st.toast("Physics parameters updated!", icon="⚙️")

st.divider()

# ===================== SYSTEM STATUS =====================
st.header("📊 System Status")

status_cols = st.columns(4)
with status_cols[0]:
    st.metric("Simulation", "Running" if st.session_state.sim_running else "Stopped")
with status_cols[1]:
    st.metric("Total Impulses", st.session_state.impulse_count)
with status_cols[2]:
    st.metric("Current Force X", f"{st.session_state.force_x:.2f}")
with status_cols[3]:
    st.metric("Current Force Y", f"{st.session_state.force_y:.2f}")

st.divider()

# ===================== INFORMATION PANEL =====================
with st.expander("📚 System Information & Physics"):
    st.markdown("""
    ## 🎓 Physics Principles
    
    This simulation follows **Newtonian mechanics**:
    
    1. **Newton's First Law**: The drone stays at rest or in motion unless acted upon by a force
    2. **Newton's Second Law**: F = ma determines acceleration from applied forces
    3. **Newton's Third Law**: Drone and particles exert equal/opposite forces
    
    ### Force Impulse System
    
    - **Impulse**: Change in momentum (Δp = F × Δt)
    - **Velocity Change**: Δv = Impulse / mass
    - **Energy Tracking**: Input energy vs system energy + heat dissipation
    
    ### Interactions
    
    - **Lennard-Jones Potential**: Particle-particle and drone-particle forces
    - **Spatial Grid**: Efficient O(n) neighbor detection
    - **Boundary Conditions**: Elastic wall collisions
    - **Damping**: Energy dissipation modeling air resistance
    
    ### Drone Properties
    
    - **Mass**: 50.0 kg (heavy - requires strong forces)
    - **Sigma**: 30.0 (large interaction radius)
    - **Epsilon**: 50.0 (strong interaction field)
    
    ### Energy Conservation
    
    The simulation tracks:
    - **Input Energy**: Total energy added via impulses
    - **System Energy**: KE + PE of all particles
    - **Heat Dissipated**: Energy lost to damping
    - **Balance**: Input = System + Heat (should be ≈0)
    """)

with st.expander("🎮 Usage Tips"):
    st.markdown("""
    ### Getting Started
    1. Click **"Start Simulation"** to begin
    2. Wait for visualization at http://localhost:5006
    3. Set force direction (X, Y)
    4. Adjust impulse magnitude
    5. Click **"Apply Force Impulse"**
    
    ### Experimentation Ideas
    - **Circular Motion**: Apply perpendicular forces
    - **Momentum Building**: Multiple impulses in same direction
    - **Direction Changes**: Perpendicular to current velocity
    - **Braking**: Apply opposite to velocity
    - **Navigation**: Push through particle crowds
    
    ### Advanced Control
    - Use diagonal forces (e.g., X=1, Y=1) for 45° motion
    - Small magnitude + many clicks = precise control
    - Large magnitude = dramatic effects
    - Watch the energy balance to verify conservation
    """)
