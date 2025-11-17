# physics_engine.py - WITH DRONE SPEED CONTROL

import time
import numpy as np
import queue
import math
from vector import Vector2D
from grid import Grid

# --- Configuration Constants ---
PARTICLE_COUNT = 100
ENV_SIZE = (800, 600)
INITIAL_NUDGE_MAX = 5.0
MAX_VELOCITY = 400.0
TIME_STEP = 0.005
VELOCITY_DAMPING = 0.999
MIN_INTERACTION_DISTANCE = 1.0
CELL_SIZE = 60.0

# State and Transition properties
S0_MASS, S0_SIGMA, S0_EPSILON = 1.0, 15.0, 2.0
S1_MASS, S1_SIGMA, S1_EPSILON = 1.5, 20.0, 5.0
ENERGY_THRESHOLD_S1, INELASTIC_COLLISION_DAMPING = 150.0, 0.85

tunable_params = {
    's0_epsilon': S0_EPSILON,
    's1_epsilon': S1_EPSILON,
    'energy_threshold_s1': ENERGY_THRESHOLD_S1,
    'drone_mode': 'idle',
    'drone_orbit_speed': 0.02,      # NEW: Angular velocity for orbit mode
    'drone_patrol_speed': 0.01,     # NEW: Frequency for patrol mode
    'log_snapshot': False           # NEW: Flag to trigger snapshot logging
}

class Particle:
    def __init__(self, particle_id, position, velocity):
        self.id, self.position, self.velocity = particle_id, position, velocity
        self.acceleration, self.net_force = Vector2D(0, 0), Vector2D(0, 0)
        self.state, self.mass, self.sigma = 'S0', S0_MASS, S0_SIGMA
        self.epsilon = tunable_params['s0_epsilon']
        self.kinetic_energy, self.potential_energy = 0.0, 0.0
    
    def update_state(self, new_state):
        if self.state == new_state: return
        self.state = new_state
        if new_state == 'S1':
            self.mass, self.sigma, self.epsilon = S1_MASS, S1_SIGMA, tunable_params['s1_epsilon']
        else:
            self.mass, self.sigma, self.epsilon = S0_MASS, S0_SIGMA, tunable_params['s0_epsilon']

def simulation_process(data_q, param_q):
    spatial_grid = Grid(ENV_SIZE, CELL_SIZE)
    
    particles = []
    num_cols = int(math.sqrt(PARTICLE_COUNT * ENV_SIZE[0] / ENV_SIZE[1]))
    num_rows = int(math.ceil(PARTICLE_COUNT / num_cols))
    spacing_x, spacing_y = ENV_SIZE[0] / (num_cols + 1), ENV_SIZE[1] / (num_rows + 1)
    for i in range(PARTICLE_COUNT):
        row, col = i // num_cols, i % num_cols
        pos = Vector2D((col + 1) * spacing_x, (row + 1) * spacing_y)
        vel = Vector2D(np.random.uniform(-INITIAL_NUDGE_MAX, INITIAL_NUDGE_MAX), 
                      np.random.uniform(-INITIAL_NUDGE_MAX, INITIAL_NUDGE_MAX))
        particles.append(Particle(i, pos, vel))

    drone = particles[0]
    drone.mass, drone.sigma, drone.epsilon = 10.0, 25.0, 15.0
    
    running, tick = False, 0
    print("[Engine] Physics engine started. Waiting for command.")

    while True:
        try:
            # Check for ALL messages in queue before proceeding
            messages_processed = 0
            while messages_processed < 10:  # Limit to prevent infinite loop
                try:
                    message = param_q.get_nowait()
                    if 'command' in message:
                        if message['command'] == 'START':
                            running = True
                            print("[Engine] START command received.")
                        elif message['command'] == 'STOP':
                            running = False
                            print("[Engine] STOP command received.")
                    elif 'params' in message:
                        tunable_params.update(message['params'])
                        print(f"[Engine] Reloaded parameters: {tunable_params}")
                    messages_processed += 1
                except queue.Empty:
                    break  # No more messages

            if not running:
                time.sleep(0.05)
                continue

            # Update drone position based on mode with controllable speed
            if tunable_params['drone_mode'] == 'orbit':
                center = Vector2D(ENV_SIZE[0] / 2, ENV_SIZE[1] / 2)
                radius = 200
                ang_vel = tunable_params['drone_orbit_speed']  # Use tunable parameter
                drone.position.x = center.x + radius * math.cos(tick * ang_vel)
                drone.position.y = center.y + radius * math.sin(tick * ang_vel)
                drone.velocity = Vector2D(0, 0)
                drone.acceleration = Vector2D(0, 0)
            elif tunable_params['drone_mode'] == 'patrol':
                amplitude = ENV_SIZE[0] / 2 - 50
                freq = tunable_params['drone_patrol_speed']  # Use tunable parameter
                drone.position.x = ENV_SIZE[0] / 2 + amplitude * math.sin(tick * freq)
                drone.position.y = ENV_SIZE[1] / 2
                drone.velocity = Vector2D(0, 0)
                drone.acceleration = Vector2D(0, 0)
            else:  # idle
                center_pos = Vector2D(ENV_SIZE[0] / 2, ENV_SIZE[1] / 2)
                jitter_strength = 0.5
                random_jitter = Vector2D(np.random.uniform(-jitter_strength, jitter_strength),
                                         np.random.uniform(-jitter_strength, jitter_strength))
                drone.position = center_pos + random_jitter
                drone.velocity = Vector2D(0, 0)
                drone.acceleration = Vector2D(0, 0)
            
            # Update particle positions (regular particles only)
            for p in particles:
                if p.state == 'S0': 
                    p.epsilon = tunable_params['s0_epsilon']
                if p.id != 0: 
                    p.position += p.velocity * TIME_STEP + p.acceleration * (0.5 * TIME_STEP**2)
                    p.velocity += p.acceleration * (0.5 * TIME_STEP)

            # Spatial grid and force calculations
            spatial_grid.clear()
            for p in particles:
                spatial_grid.insert(p)
                p.net_force, p.potential_energy = Vector2D(0, 0), 0.0
            
            total_nn_dist = 0.0
            
            # Drone interactions
            for p1 in particles:
                if p1.id == 0: continue
                dist_vec = drone.position - p1.position
                r = dist_vec.magnitude()
                r_clamped = max(r, MIN_INTERACTION_DISTANCE)
                sigma_avg = (p1.sigma + drone.sigma) / 2.0
                epsilon_avg = math.sqrt(p1.epsilon * drone.epsilon)
                sr6 = (sigma_avg / r_clamped)**6
                sr12 = sr6 * sr6
                potential = 4.0 * epsilon_avg * (sr12 - sr6)
                force_mag = 24.0 * epsilon_avg * (2.0 * sr12 - sr6) / r_clamped
                p1.net_force -= dist_vec.normalize() * force_mag
                p1.potential_energy += potential
                
                # Particle-particle interactions
                potential_neighbors = spatial_grid.get_neighbors(p1)
                min_dist_sq = float('inf')
                for p2 in potential_neighbors:
                    if p1.id >= p2.id: continue
                    dist_vec = p2.position - p1.position
                    r = dist_vec.magnitude()
                    r_clamped = max(r, MIN_INTERACTION_DISTANCE)
                    sigma_avg = (p1.sigma + p2.sigma) / 2.0
                    epsilon_avg = math.sqrt(p1.epsilon * p2.epsilon)
                    sr6 = (sigma_avg / r_clamped)**6
                    potential = 4.0 * epsilon_avg * (sr6*sr6 - sr6)
                    force_mag = 24.0 * epsilon_avg * (2.0 * sr6*sr6 - sr6) / r_clamped
                    force_vec = dist_vec.normalize() * force_mag
                    p1.net_force -= force_vec
                    p2.net_force += force_vec
                    p1.potential_energy += potential / 2.0
                    p2.potential_energy += potential / 2.0
                    dist_sq = r*r
                    if dist_sq < min_dist_sq: min_dist_sq = dist_sq
                if min_dist_sq != float('inf'):
                    total_nn_dist += math.sqrt(min_dist_sq)

            # Update velocities and check state transitions
            for p in particles:
                if p.id == 0: continue
                p.acceleration = p.net_force / p.mass
                p.velocity += p.acceleration * (0.5 * TIME_STEP)
                p.velocity *= VELOCITY_DAMPING
                speed = p.velocity.magnitude()
                if speed > MAX_VELOCITY: 
                    p.velocity = p.velocity.normalize() * MAX_VELOCITY
                p.kinetic_energy = 0.5 * p.mass * p.velocity.magnitude()**2
                if math.isnan(p.kinetic_energy):
                    p.velocity, p.kinetic_energy = Vector2D(0,0), 0
                
                total_energy = p.kinetic_energy + p.potential_energy
                if p.state == 'S0' and total_energy > tunable_params['energy_threshold_s1']: 
                    p.update_state('S1')
                elif p.state == 'S1' and total_energy < tunable_params['energy_threshold_s1']: 
                    p.velocity *= INELASTIC_COLLISION_DAMPING
                    p.update_state('S0')

            # Boundary conditions
            for p in particles:
                if p.position.x < 0 or p.position.x > ENV_SIZE[0]: 
                    p.velocity.x *= -1
                    p.position.x = np.clip(p.position.x, 0, ENV_SIZE[0])
                if p.position.y < 0 or p.position.y > ENV_SIZE[1]: 
                    p.velocity.y *= -1
                    p.position.y = np.clip(p.position.y, 0, ENV_SIZE[1])
            
            # Package data for visualization
            num_reg_particles = PARTICLE_COUNT - 1
            agent_packet = {
                'x': np.array([p.position.x for p in particles if p.id != 0], dtype=np.float32),
                'y': np.array([p.position.y for p in particles if p.id != 0], dtype=np.float32),
                'ke': np.array([p.kinetic_energy for p in particles if p.id != 0], dtype=np.float32),
                'mass': np.array([p.mass for p in particles if p.id != 0], dtype=np.float32),
                'outline_color_num': np.array([1 if p.state == 'S1' else 0 for p in particles if p.id != 0], dtype=np.uint8),
                'drone_x': drone.position.x,
                'drone_y': drone.position.y
            }

            total_ke = np.sum(agent_packet['ke'])
            total_pe = sum(p.potential_energy for p in particles if p.id != 0)
            system_temperature = total_ke / num_reg_particles if num_reg_particles > 0 else 0
            system_order = total_nn_dist / num_reg_particles if num_reg_particles > 0 else 0
            
            metrics_packet = {
                'total_ke': total_ke, 'total_pe': total_pe, 'total_energy': total_ke + total_pe,
                's1_count': np.sum(agent_packet['outline_color_num']),
                'temperature': system_temperature, 'order': system_order
            }
            data_point = {'tick': tick, 'agents': agent_packet, 'metrics': metrics_packet}
        
            try: 
                data_q.put_nowait(data_point)
            except queue.Full: 
                pass

            tick += 1
            time.sleep(TIME_STEP)

        except Exception as e:
            print(f"[Engine] CRITICAL ERROR in simulation loop at tick {tick}: {e}")
            import traceback
            traceback.print_exc()
            running = False
