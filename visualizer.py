# visualizer.py - WITH PROPER CLEANUP

import queue
import numpy as np
from bokeh.plotting import figure, curdoc
from bokeh.models import ColumnDataSource, LinearColorMapper
from bokeh.layouts import layout, column, row
from bokeh.server.server import Server
from bokeh.palettes import Viridis256
from collections import deque
import signal
import sys

def visualization_process(data_q):
    print("[Visualizer] Process started.")
    
    server = None
    
    def cleanup_and_exit(signum=None, frame=None):
        """Cleanup function to properly stop the Bokeh server"""
        print("[Visualizer] Cleaning up...")
        if server:
            try:
                server.stop()
                print("[Visualizer] Server stopped.")
            except:
                pass
        sys.exit(0)
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, cleanup_and_exit)
    signal.signal(signal.SIGINT, cleanup_and_exit)
    
    def bokeh_app(doc):
        # Data Sources
        agent_source = ColumnDataSource(data=dict(x=[], y=[], color=[], size=[], outline_color=[]))
        drone_source = ColumnDataSource(data=dict(x=[], y=[]))
        metrics_source = ColumnDataSource(data=dict(tick=[], ke=[], pe=[], total_e=[]))
        hist_source = ColumnDataSource(data=dict(top=[], bottom=[], left=[], right=[]))
        temp_source = ColumnDataSource(data=dict(tick=[], temp=[]))
        order_source = ColumnDataSource(data=dict(tick=[], order=[]))

        color_mapper = LinearColorMapper(palette=Viridis256, low=0, high=150)
        
        # Define the color mapping for the received numbers
        outline_color_map = ['#555555', '#FFFFFF']

        # Plot 1: Main Particle Simulation
        p1 = figure(height=600, width=800, title="Particle Positions & Kinetic Energy", 
                   x_range=(0, 800), y_range=(0, 600), match_aspect=True, 
                   tools="pan,wheel_zoom,reset,save")
        p1.background_fill_color = "#222222"
        p1.scatter(x='x', y='y', size='size', color={'field': 'color', 'transform': color_mapper}, 
                  line_color='outline_color', line_width=2, marker='circle', source=agent_source)
        p1.scatter(x='x', y='y', size=25, color="#E11D48", line_color="white", 
                  line_width=2, marker='circle', source=drone_source)

        # Plot 2: System Energy Homeostasis
        p2 = figure(height=300, width=600, title="System Energy Homeostasis", 
                   x_axis_label="Tick", y_axis_label="Energy", tools="")
        p2.line(x='tick', y='ke', source=metrics_source, legend_label="Kinetic Energy", color="#F59E0B")
        p2.line(x='tick', y='pe', source=metrics_source, legend_label="Potential Energy", color="#3B82F6")
        p2.line(x='tick', y='total_e', source=metrics_source, legend_label="Total Energy", 
               color="#FFFFFF", line_width=2)
        p2.legend.location = "top_left"
        
        # Plot 3: System Order
        p5 = figure(height=300, width=600, title="System Order (Avg. Nearest Neighbor Distance)", 
                   x_axis_label="Tick", y_axis_label="Avg. Distance", tools="")
        p5.line(x='tick', y='order', source=order_source, color="#10B981", line_width=2)

        # Plot 4: System Temperature
        p4 = figure(height=300, width=600, title="System Temperature", 
                   x_axis_label="Tick", y_axis_label="Temperature (Avg. KE)", tools="")
        p4.line(x='tick', y='temp', source=temp_source, color="#EF4444", line_width=2)
        
        # Data History Deques
        history_len = 1000
        ke_history, pe_history, total_e_history, temp_history, order_history = (
            deque(maxlen=history_len) for _ in range(5)
        )

        def update():
            data = None
            
            # Drain the queue - keep only the last frame
            try:
                while True:
                    data = data_q.get_nowait()
            except queue.Empty:
                pass

            # If we didn't get any data, just return
            if data is None:
                return
            
            try:
                agent_data, metrics_data = data['agents'], data['metrics']

                # Convert the numeric color codes back to hex strings for Bokeh
                outline_colors = [outline_color_map[num] for num in agent_data['outline_color_num']]

                agent_source.data = {
                    'x': agent_data['x'], 
                    'y': agent_data['y'], 
                    'color': agent_data['ke'], 
                    'size': agent_data['mass'] * 8,
                    'outline_color': outline_colors
                }
                drone_source.data = {
                    'x': [agent_data['drone_x']], 
                    'y': [agent_data['drone_y']]
                }

                ke_history.append(metrics_data['total_ke'])
                pe_history.append(metrics_data['total_pe'])
                total_e_history.append(metrics_data['total_energy'])
                metrics_source.data = {
                    'tick': list(range(len(ke_history))), 
                    'ke': list(ke_history), 
                    'pe': list(pe_history), 
                    'total_e': list(total_e_history)
                }
                
                temp_history.append(metrics_data['temperature'])
                temp_source.data = {
                    'tick': list(range(len(temp_history))), 
                    'temp': list(temp_history)
                }
                
                order_history.append(metrics_data['order'])
                order_source.data = {
                    'tick': list(range(len(order_history))), 
                    'order': list(order_history)
                }

            except Exception as e:
                print(f"[Visualizer] Error processing data: {e}")
                pass

        doc.add_periodic_callback(update, 50)
        plot_layout = row(p1, column(p2, p5, p4))
        doc.add_root(plot_layout)
        doc.title = "N-Body Homeostasis Simulation"

    try:
        server = Server({'/': bokeh_app}, num_procs=1, port=5006, 
                       allow_websocket_origin=["*"])
        server.start()
        print("[Visualizer] Bokeh server started at http://localhost:5006")
        server.io_loop.start()
    except OSError as e:
        if "Address already in use" in str(e):
            print("[Visualizer] ERROR: Port 5006 is already in use. Please wait a moment and try again.")
            print("[Visualizer] If problem persists, kill the process using: lsof -ti:5006 | xargs kill -9")
        else:
            print(f"[Visualizer] ERROR: {e}")
    except Exception as e:
        print(f"[Visualizer] Unexpected error: {e}")
    finally:
        cleanup_and_exit()
