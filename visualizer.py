"""
Bokeh-based visualization for the physics simulation.
Displays particles, drone, and real-time metrics.
"""

import queue
import signal
import sys
from collections import deque

from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, LinearColorMapper
from bokeh.layouts import row, column
from bokeh.server.server import Server
from bokeh.palettes import Viridis256

def visualization_process(data_q):
    """Main visualization process that runs Bokeh server"""
    print("[Visualizer] Process started.")
    
    server = None
    
    def cleanup_and_exit(signum=None, frame=None):
        """Clean shutdown handler"""
        print("[Visualizer] Cleaning up...")
        if server:
            try:
                server.stop()
                print("[Visualizer] Server stopped.")
            except:
                pass
        sys.exit(0)
    
    # Register signal handlers
    signal.signal(signal.SIGTERM, cleanup_and_exit)
    signal.signal(signal.SIGINT, cleanup_and_exit)
    
    def bokeh_app(doc):
        """Bokeh application definition"""
        
        # Data sources
        agent_source = ColumnDataSource(data={
            'x': [], 'y': [], 'color': [], 'size': [], 'outline_color': []
        })
        drone_source = ColumnDataSource(data={'x': [], 'y': []})
        metrics_source = ColumnDataSource(data={
            'tick': [], 'ke': [], 'pe': [], 'total_e': []
        })
        temp_source = ColumnDataSource(data={'tick': [], 'temp': []})
        energy_accounting_source = ColumnDataSource(data={
            'tick': [], 'heat': [], 'input_energy': [], 'balance': []
        })
        momentum_source = ColumnDataSource(data={
            'tick': [], 'mom_x': [], 'mom_y': []
        })

        # Color mapper for kinetic energy
        color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=150)
        outline_color_map = ['#555555', '#FFFFFF']  # S0, S1

        # Plot 1: Main Particle Simulation
        p1 = figure(
            height=600, width=800,
            title="Particle Positions & Kinetic Energy",
            x_range=(0, 800), y_range=(0, 600),
            match_aspect=True,
            tools="pan,wheel_zoom,reset,save"
        )
        p1.background_fill_color = "#222222"
        
        # Render agents first (background)
        agents_renderer = p1.scatter(
            x='x', y='y',
            size='size',
            color={'field': 'color', 'transform': color_mapper},
            line_color='outline_color',
            line_width=2,
            marker='circle',
            source=agent_source
        )
        
        # Render drone on top (overlay)
        drone_renderer = p1.scatter(
            x='x', y='y',
            size=18,
            color="#E11D48",
            line_color="white",
            line_width=3,
            marker='circle',
            source=drone_source,
            legend_label="Drone"
        )
        
        # Set render levels
        agents_renderer.level = 'glyph'
        drone_renderer.level = 'overlay'
        p1.legend.location = "top_left"

        # Plot 2: System Energy
        p2 = figure(
            height=250, width=600,
            title="System Energy (KE + PE)",
            x_axis_label="Tick",
            y_axis_label="Energy (J)",
            tools=""
        )
        p2.line(x='tick', y='ke', source=metrics_source,
                legend_label="Kinetic", color="#F59E0B", line_width=2)
        p2.line(x='tick', y='pe', source=metrics_source,
                legend_label="Potential", color="#3B82F6", line_width=2)
        p2.line(x='tick', y='total_e', source=metrics_source,
                legend_label="Total (KE+PE)", color="#FFFFFF", line_width=2)
        p2.legend.location = "top_left"
        
        # Plot 3: Energy Conservation
        p3 = figure(
            height=250, width=600,
            title="Energy Conservation: Input = System + Heat",
            x_axis_label="Tick",
            y_axis_label="Energy (J)",
            tools=""
        )
        p3.line(x='tick', y='input_energy', source=energy_accounting_source,
                legend_label="Input (Impulses)", color="#10B981", line_width=2)
        p3.line(x='tick', y='heat', source=energy_accounting_source,
                legend_label="Heat Dissipated", color="#EF4444", line_width=2)
        p3.line(x='tick', y='balance', source=energy_accounting_source,
                legend_label="Balance Error", color="#FFFFFF", line_width=1, line_dash="dashed")
        p3.legend.location = "top_left"

        # Plot 4: Temperature
        p4 = figure(
            height=250, width=600,
            title="System Temperature (Avg KE)",
            x_axis_label="Tick",
            y_axis_label="Temperature",
            tools=""
        )
        p4.line(x='tick', y='temp', source=temp_source,
                color="#F59E0B", line_width=2)

        # Plot 5: Momentum Conservation
        p5 = figure(
            height=250, width=600,
            title="Total Momentum (Changes Only with Impulses)",
            x_axis_label="Tick",
            y_axis_label="Momentum (kg·m/s)",
            tools=""
        )
        p5.line(x='tick', y='mom_x', source=momentum_source,
                legend_label="Momentum X", color="#06B6D4", line_width=2)
        p5.line(x='tick', y='mom_y', source=momentum_source,
                legend_label="Momentum Y", color="#8B5CF6", line_width=2)
        p5.legend.location = "top_left"
        
        # Data history buffers
        HISTORY_LEN = 1000
        histories = {
            'ke': deque(maxlen=HISTORY_LEN),
            'pe': deque(maxlen=HISTORY_LEN),
            'total_e': deque(maxlen=HISTORY_LEN),
            'temp': deque(maxlen=HISTORY_LEN),
            'heat': deque(maxlen=HISTORY_LEN),
            'input': deque(maxlen=HISTORY_LEN),
            'balance': deque(maxlen=HISTORY_LEN),
            'mom_x': deque(maxlen=HISTORY_LEN),
            'mom_y': deque(maxlen=HISTORY_LEN)
        }

        def update():
            """Update visualization with latest data"""
            data = None
            
            # Get latest data (drain queue)
            try:
                while True:
                    data = data_q.get_nowait()
            except queue.Empty:
                pass

            if data is None:
                return
            
            try:
                agent_data = data['agents']
                metrics_data = data['metrics']

                # Map state to outline colors
                outline_colors = [
                    outline_color_map[num] for num in agent_data['outline_color_num']
                ]

                # Update particle data
                agent_source.data = {
                    'x': agent_data['x'],
                    'y': agent_data['y'],
                    'color': agent_data['ke'],
                    'size': agent_data['mass'] * 8,
                    'outline_color': outline_colors
                }
                
                # Update drone data
                drone_source.data = {
                    'x': [agent_data['drone_x']],
                    'y': [agent_data['drone_y']]
                }

                # Update energy histories
                histories['ke'].append(metrics_data['total_ke'])
                histories['pe'].append(metrics_data['total_pe'])
                histories['total_e'].append(metrics_data['total_energy'])
                
                metrics_source.data = {
                    'tick': list(range(len(histories['ke']))),
                    'ke': list(histories['ke']),
                    'pe': list(histories['pe']),
                    'total_e': list(histories['total_e'])
                }
                
                # Update temperature
                histories['temp'].append(metrics_data['temperature'])
                temp_source.data = {
                    'tick': list(range(len(histories['temp']))),
                    'temp': list(histories['temp'])
                }
                
                # Update energy accounting
                histories['heat'].append(metrics_data['heat_dissipated'])
                histories['input'].append(metrics_data['energy_input'])
                histories['balance'].append(metrics_data['energy_balance'])
                
                energy_accounting_source.data = {
                    'tick': list(range(len(histories['heat']))),
                    'heat': list(histories['heat']),
                    'input_energy': list(histories['input']),
                    'balance': list(histories['balance'])
                }
                
                # Update momentum
                histories['mom_x'].append(metrics_data['momentum_x'])
                histories['mom_y'].append(metrics_data['momentum_y'])
                
                momentum_source.data = {
                    'tick': list(range(len(histories['mom_x']))),
                    'mom_x': list(histories['mom_x']),
                    'mom_y': list(histories['mom_y'])
                }

            except KeyError as e:
                print(f"[Visualizer] Missing data key: {e}")
            except Exception as e:
                print(f"[Visualizer] Error processing data: {e}")

        # Add periodic update callback
        doc.add_periodic_callback(update, 50)  # 50ms = 20 FPS
        
        # Create layout
        plot_layout = row(p1, column(p2, p3, p4, p5))
        doc.add_root(plot_layout)
        doc.title = "Physics-Based Drone Simulation"

    # Start Bokeh server
    try:
        server = Server(
            {'/': bokeh_app},
            num_procs=1,
            port=5006,
            allow_websocket_origin=["*"]
        )
        server.start()
        print("[Visualizer] Bokeh server started at http://localhost:5006")
        server.io_loop.start()
        
    except OSError as e:
        if "Address already in use" in str(e):
            print("[Visualizer] ERROR: Port 5006 is already in use.")
            print("[Visualizer] Kill the process using: lsof -ti:5006 | xargs kill -9")
        else:
            print(f"[Visualizer] ERROR: {e}")
    except Exception as e:
        print(f"[Visualizer] Unexpected error: {e}")
    finally:
        cleanup_and_exit()
