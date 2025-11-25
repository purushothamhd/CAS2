# physics_engine.py - PURE PHYSICS-BASED DRONE WITH FORCE IMPULSES

import time
import numpy as np
import queue
import math
import csv
from datetime import datetime
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
ENERGY_THRESHOLD_S1 = 150.0
INELASTIC_COLLISION_DAMPING = 0.85

# Drone Physics Properties
DRONE_MASS = 1.0
DRONE_SIGMA = 30.0
DRONE_EPSILON = 50.0

# Tunable parameters
tunable_params = {
    's0_epsilon': S0_EPSILON,
    's1_epsilon': S1_EPSILON,
    'energy_threshold_s1': ENERGY_THRESHOLD_S1,
    'impulse_magnitude': 5000.0,
}

class Particle:
    def __init__(self, particle_id, position, velocity):
        self.id = particle_id
        self.position = position
        self.velocity = velocity
        self.acceleration = Vector2D(0, 0)
        self.net_force = Vector2D(0, 0)
        self.state = 'S0'
        self.mass = S0_MASS
        self.sigma = S0_SIGMA
        self.epsilon = tunable_params['s0_epsilon']
        self.kinetic_energy = 0.0
        self.potential_energy = 0.0
    
    def update_state(self, new_state):
        if self.state == new_state:
            return
        self.state = new_state
        if new_state == 'S1':
            self.mass = S1_MASS
            self.sigma = S1_SIGMA
            self.epsilon = tunable_params['s1_epsilon']
        else:
            self.mass = S0_MASS
            self.sigma = S0_SIGMA
            self.epsilon = tunable_params['s0_epsilon']

def simulation_process(data_q, param_q):
    """Main simulation loop with physics calculations and energy tracking"""
    spatial_grid = Grid(ENV_SIZE, CELL_SIZE)
    
    # Initialize particles in a grid layout to avoid initial explosions
    particles = []
    num_cols = int(math.sqrt(PARTICLE_COUNT * ENV_SIZE[0] / ENV_SIZE[1]))
    num_rows = int(math.ceil(PARTICLE_COUNT / num_cols))
    spacing_x = ENV_SIZE[0] / (num_cols + 1)
    spacing_y = ENV_SIZE[1] / (num_rows + 1)
    
    for i in range(PARTICLE_COUNT):
        row = i // num_cols
        col = i % num_cols
        pos = Vector2D((col + 1) * spacing_x, (row + 1) * spacing_y)
        vel = Vector2D(
            np.random.uniform(-INITIAL_NUDGE_MAX, INITIAL_NUDGE_MAX),
            np.random.uniform(-INITIAL_NUDGE_MAX, INITIAL_NUDGE_MAX)
        )
        particles.append(Particle(i, pos, vel))

    # Configure Drone (Particle 0) - Starts at center, stationary
    drone = particles[0]
    drone.mass = DRONE_MASS
    drone.sigma = DRONE_SIGMA
    drone.epsilon = DRONE_EPSILON
    drone.position = Vector2D(ENV_SIZE[0] / 2, ENV_SIZE[1] / 2)
    drone.velocity = Vector2D(0, 0)
    
    # Energy tracking
    total_energy_input = 0.0  # Total energy added via impulses
    total_heat_dissipated = 0.0  # Energy lost due to damping
    
    running = False
    tick = 0

    print("[Engine] Physics engine started. Drone ready for force impulses.")
    
    def log_snapshot(particles, tick):
        """Save current simulation state to CSV"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"simulation_snapshot_tick_{tick}_{timestamp}.csv"
        try:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = [
                    'tick', 'particle_id', 'is_drone', 'state',
                    'position_x', 'position_y', 'velocity_x', 'velocity_y',
                    'mass', 'kinetic_energy', 'potential_energy', 'total_energy'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for p in particles:
                    total_e = p.kinetic_energy + p.potential_energy
                    writer.writerow({
                        'tick': tick,
                        'particle_id': p.id,
                        'is_drone': 'Yes' if p.id == 0 else 'No',
                        'state': p.state,
                        'position_x': round(p.position.x, 4),
                        'position_y': round(p.position.y, 4),
                        'velocity_x': round(p.velocity.x, 4),
                        'velocity_y': round(p.velocity.y, 4),
                        'mass': p.mass,
                        'kinetic_energy': round(p.kinetic_energy, 4),
                        'potential_energy': round(p.potential_energy, 4),
                        'total_energy': round(total_e, 4)
                    })
            print(f"[Engine] Snapshot saved: {filename}")
            return filename
        except Exception as e:
            print(f"[Engine] Snapshot error: {e}")
            return None

    while True:
        try:
            # Process incoming messages
            messages_processed = 0
            while messages_processed < 10:
                try:
                    message = param_q.get_nowait()
                    
                    if 'command' in message:
                        cmd = message['command']
                        
                        if cmd == 'START':
                            running = True
                            print("[Engine] Simulation started")
                        
                        elif cmd == 'STOP':
                            running = False
                            print("[Engine] Simulation stopped")
                        
                        elif cmd == 'LOG_SNAPSHOT':
                            log_snapshot(particles, tick)
                        
                        elif cmd == 'APPLY_IMPULSE':
                            # Apply force impulse to drone
                            force_x = message.get('force_x', 0.0)
                            force_y = message.get('force_y', 0.0)
                            magnitude = tunable_params['impulse_magnitude']
                            
                            force_vec = Vector2D(force_x, force_y)
                            force_magnitude = force_vec.magnitude()
                            
                            if force_magnitude > 0:
                                # Normalize and scale to desired magnitude
                                impulse = force_vec.normalize() * magnitude
                                
                                # Apply impulse: Δv = Impulse / m
                                delta_v = impulse / drone.mass
                                drone.velocity += delta_v
                                
                                # Track energy input (KE = 0.5 * m * v²)
                                old_ke = 0.5 * drone.mass * (drone.velocity - delta_v).magnitude()**2
                                new_ke = 0.5 * drone.mass * drone.velocity.magnitude()**2
                                energy_added = new_ke - old_ke
                                total_energy_input += energy_added
                                
                                print(f"[Engine] Impulse applied: direction=({force_x:.2f}, {force_y:.2f}), "
                                      f"magnitude={magnitude:.1f}, Δv=({delta_v.x:.2f}, {delta_v.y:.2f}), "
                                      f"energy_added={energy_added:.2f}J")
                            else:
                                print("[Engine] Zero force vector - no impulse applied")
                    
                    elif 'params' in message:
                        tunable_params.update(message['params'])
                        print(f"[Engine] Parameters updated: {message['params']}")
                    
                    messages_processed += 1
                    
                except queue.Empty:
                    break

            if not running:
                time.sleep(0.05)
                continue

            # --- PHYSICS INTEGRATION ---
            
            # Store energy before damping for heat calculation
            total_ke_before_damping = sum(0.5 * p.mass * p.velocity.magnitude()**2 for p in particles)
            
            # Position update (Euler integration)
            for p in particles:
                if p.state == 'S0':
                    p.epsilon = tunable_params['s0_epsilon']
                
                p.position += p.velocity * TIME_STEP + p.acceleration * (0.5 * TIME_STEP**2)
                p.velocity += p.acceleration * (0.5 * TIME_STEP)

            # --- SPATIAL GRID & FORCE CALCULATION ---
            spatial_grid.clear()
            for p in particles:
                spatial_grid.insert(p)
                p.net_force = Vector2D(0, 0)
                p.potential_energy = 0.0
            
            total_nn_dist = 0.0
            
            # 1. Drone-Particle Interactions (Newton's 3rd Law)
            for p1 in particles:
                if p1.id == 0:
                    continue
                
                dist_vec = p1.position - drone.position
                r = dist_vec.magnitude()
                r_clamped = max(r, MIN_INTERACTION_DISTANCE)
                
                # Mixed interaction properties
                sigma_avg = (p1.sigma + drone.sigma) / 2.0
                epsilon_avg = math.sqrt(p1.epsilon * drone.epsilon)
                
                # Lennard-Jones Force
                sr6 = (sigma_avg / r_clamped)**6
                sr12 = sr6 * sr6
                potential = 4.0 * epsilon_avg * (sr12 - sr6)
                force_mag = 24.0 * epsilon_avg * (2.0 * sr12 - sr6) / r_clamped
                
                force_vec = dist_vec.normalize() * force_mag
                
                # Apply forces (Newton's 3rd Law)
                p1.net_force += force_vec
                p1.potential_energy += potential
                drone.net_force -= force_vec

            # 2. Particle-Particle Interactions
            for p1 in particles:
                if p1.id == 0:
                    continue
                
                potential_neighbors = spatial_grid.get_neighbors(p1)
                min_dist_sq = float('inf')
                
                for p2 in potential_neighbors:
                    if p1.id >= p2.id or p2.id == 0:
                        continue
                    
                    dist_vec = p2.position - p1.position
                    r = dist_vec.magnitude()
                    r_clamped = max(r, MIN_INTERACTION_DISTANCE)
                    
                    sigma_avg = (p1.sigma + p2.sigma) / 2.0
                    epsilon_avg = math.sqrt(p1.epsilon * p2.epsilon)
                    
                    sr6 = (sigma_avg / r_clamped)**6
                    sr12 = sr6 * sr6
                    force_mag = 24.0 * epsilon_avg * (2.0 * sr12 - sr6) / r_clamped
                    force_vec = dist_vec.normalize() * force_mag
                    
                    p1.net_force -= force_vec
                    p2.net_force += force_vec
                    
                    potential = 4.0 * epsilon_avg * (sr12 - sr6)
                    p1.potential_energy += potential / 2.0
                    p2.potential_energy += potential / 2.0
                    
                    if r * r < min_dist_sq:
                        min_dist_sq = r * r
                
                if min_dist_sq != float('inf'):
                    total_nn_dist += math.sqrt(min_dist_sq)

            # --- VELOCITY UPDATE & STATE TRANSITIONS ---
            for p in particles:
                # Calculate acceleration (F = ma)
                p.acceleration = p.net_force / p.mass
                p.velocity += p.acceleration * (0.5 * TIME_STEP)
                
                # Apply damping (air resistance)
                p.velocity *= VELOCITY_DAMPING
                
                # Speed limit
                speed = p.velocity.magnitude()
                if speed > MAX_VELOCITY:
                    p.velocity = p.velocity.normalize() * MAX_VELOCITY
                
                p.kinetic_energy = 0.5 * p.mass * speed**2
                
                # State transitions (only for regular particles, not drone)
                if p.id != 0:
                    total_energy = p.kinetic_energy + p.potential_energy
                    if p.state == 'S0' and total_energy > tunable_params['energy_threshold_s1']:
                        p.update_state('S1')
                    elif p.state == 'S1' and total_energy < tunable_params['energy_threshold_s1']:
                        p.velocity *= INELASTIC_COLLISION_DAMPING
                        p.update_state('S0')

            # Calculate heat dissipated due to damping
            total_ke_after_damping = sum(0.5 * p.mass * p.velocity.magnitude()**2 for p in particles)
            heat_this_step = total_ke_before_damping - total_ke_after_damping
            total_heat_dissipated += heat_this_step

            # --- BOUNDARY CONDITIONS ---
            for p in particles:
                radius = p.sigma / 2.0
                
                # X-axis boundaries
                if p.position.x - radius < 0:
                    p.velocity.x *= -1
                    p.position.x = radius
                elif p.position.x + radius > ENV_SIZE[0]:
                    p.velocity.x *= -1
                    p.position.x = ENV_SIZE[0] - radius
                
                # Y-axis boundaries
                if p.position.y - radius < 0:
                    p.velocity.y *= -1
                    p.position.y = radius
                elif p.position.y + radius > ENV_SIZE[1]:
                    p.velocity.y *= -1
                    p.position.y = ENV_SIZE[1] - radius

            # --- CALCULATE TOTAL MOMENTUM ---
            total_momentum_x = sum(p.mass * p.velocity.x for p in particles)
            total_momentum_y = sum(p.mass * p.velocity.y for p in particles)

            # --- PACKAGE DATA FOR VISUALIZATION ---
            num_reg_particles = PARTICLE_COUNT - 1
            
            agent_packet = {
                'x': np.array([p.position.x for p in particles if p.id != 0], dtype=np.float32),
                'y': np.array([p.position.y for p in particles if p.id != 0], dtype=np.float32),
                'ke': np.array([p.kinetic_energy for p in particles if p.id != 0], dtype=np.float32),
                'mass': np.array([p.mass for p in particles if p.id != 0], dtype=np.float32),
                'outline_color_num': np.array([1 if p.state == 'S1' else 0 for p in particles if p.id != 0], dtype=np.uint8),
                'drone_x': drone.position.x,
                'drone_y': drone.position.y,
                'drone_vx': drone.velocity.x,
                'drone_vy': drone.velocity.y
            }

            total_ke = np.sum(agent_packet['ke']) + drone.kinetic_energy
            total_pe = sum(p.potential_energy for p in particles)
            total_system_energy = total_ke + total_pe
            
            # Energy balance: Input Energy = System Energy + Heat Dissipated
            energy_balance = total_energy_input - (total_system_energy + total_heat_dissipated)
            
            metrics_packet = {
                'total_ke': total_ke,
                'total_pe': total_pe,
                'total_energy': total_system_energy,
                's1_count': np.sum(agent_packet['outline_color_num']),
                'temperature': total_ke / PARTICLE_COUNT if PARTICLE_COUNT > 0 else 0,
                'order': total_nn_dist / num_reg_particles if num_reg_particles > 0 else 0,
                'drone_speed': drone.velocity.magnitude(),
                'drone_ke': drone.kinetic_energy,
                'heat_dissipated': total_heat_dissipated,
                'energy_input': total_energy_input,
                'energy_balance': energy_balance,
                'momentum_x': total_momentum_x,
                'momentum_y': total_momentum_y
            }
            
            try:
                data_q.put_nowait({'tick': tick, 'agents': agent_packet, 'metrics': metrics_packet})
            except queue.Full:
                pass

            tick += 1
            time.sleep(TIME_STEP)

        except Exception as e:
            print(f"[Engine] ERROR: {e}")
            import traceback
            traceback.print_exc()
            running = False
