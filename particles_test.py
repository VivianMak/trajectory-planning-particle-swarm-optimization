import numpy as np
import matplotlib.pyplot as plt
from trajectory import MultiAxisTrajectoryGenerator
import time
from simulator.helper_fcns.utils import EndEffector
import math
from simulator.helper_fcns.utils import wraptopi
from tkinter import ttk

class Particle:
    """Represents a particle in the PSO algorithm for trajectory optimization.
    
    Each particle represents a set of trajectory parameters: t1 (acceleration end time),
    t2 (constant velocity end time), and v_max (maximum velocity).
    """

    def __init__(self, pos=None, vel=None, bounds=None):
        """Initialize a new particle.
        
        Args:
            pos: Initial position [t1, t2, v_max] or None for random initialization
            vel: Initial velocity or None for random initialization
            bounds: Bounds for particle parameters [(t1_min, t1_max), (t2_min, t2_max), (v_max_min, v_max_max)]
        """
        self.bounds = bounds or [(0.1, 0.4), (0.6, 0.9), (0.1, 2.0)]
        
        # Initialize with random values if not provided
        if pos is None:
            # [t1, t2, v_max]
            t1 = np.random.uniform(self.bounds[0][0], self.bounds[0][1])
            t2 = np.random.uniform(self.bounds[1][0], self.bounds[1][1])
            v_max = np.random.uniform(self.bounds[2][0], self.bounds[2][1])
            self.pos = np.array([t1, t2, v_max])
        else:
            self.pos = np.array(pos)
            
        if vel is None:
            # Small random initial velocities
            self.vel = np.random.uniform(-0.05, 0.05, 3)
        else:
            self.vel = np.array(vel)
            
        self.fitness = None
        self.p_best = self.pos.copy()
        self.p_best_fitness = None


class PSO_TrajectoryOptimizer:
    """Particle Swarm Optimization for trajectory parameters optimization.
    
    This optimizer finds the optimal parameters (t1, t2, v_max) for a trapezoidal
    velocity profile between waypoints, considering metrics like minimum jerk,
    minimum execution time, or a combination of both.
    """
    
    def __init__(self, start_pos, final_pos, metric="min_jerk", n_particles=50, 
                 iterations=100, a_max=10.0, v_max=5.0, ndof=5):
        """Initialize the PSO trajectory optimizer.
        
        Args:
            start_pos: Starting position/joint values
            final_pos: Ending position/joint values
            metric: Optimization metric ("min_jerk", "min_time", "combined")
            n_particles: Number of particles in the swarm
            iterations: Maximum number of iterations
            a_max: Maximum acceleration constraint
            v_max: Maximum velocity constraint
            ndof: Number of degrees of freedom
        """
        self.start_pos = np.array(start_pos)
        self.final_pos = np.array(final_pos)
        self.metric = metric
        self.n_particles = n_particles
        self.iterations = iterations
        self.a_max_limit = a_max
        self.v_max_limit = v_max
        self.ndof = ndof
        
        # Total Distance to Cover
        self.distance = np.abs(self.final_pos - self.start_pos)
        
        # Parameter bounds for t1, t2, v_max
        self.bounds = [(0.1, 0.4), (0.6, 0.9), (0.1, self.v_max_limit)]
        
        # Particle swarm attributes
        self.particles = []
        
        # PSO algorithm parameters
        self.alpha = 0.5      # Random Disturbance Coefficient
        self.w_min = 0.4      # Minimum inertia weight
        self.w_max = 0.9      # Maximum inertia weight
        self.c1 = 2.0         # Cognitive learning factor
        self.c2 = 2.0         # Social learning factor
        
        # Best solution found
        self.global_best = None
        self.global_best_fitness = None
        
        # Fitness history for tracking progress
        self.fitness_history = []
    
    def initialize_particles(self):
        """Initialize the particle swarm with random parameters."""
        self.particles = []
        for _ in range(self.n_particles):
            particle = Particle(bounds=self.bounds)
            particle.fitness = self.calculate_fitness(particle)
            particle.p_best_fitness = particle.fitness
            
            self.particles.append(particle)
            
            # Update global best if needed
            if self.global_best is None or particle.fitness > self.global_best_fitness:
                self.global_best = particle.pos.copy()
                self.global_best_fitness = particle.fitness
    
    def calculate_fitness(self, particle):
        """
        Fitness evaluation with soft quadratic penalties instead of -inf.
        The function still returns 'higher is better'.
        """

        t1, t2, v_max = particle.pos
        accel_time  = t1
        decel_time  = 1.0 - t2
        const_time  = t2 - t1

        EPS = 1e-6
        accel_time  = max(accel_time,  EPS)
        decel_time  = max(decel_time,  EPS)


        max_accel   = v_max / accel_time
        jerk_metric = max_accel / min(accel_time, decel_time)

        if self.metric == "min_jerk":
            base_fitness = -jerk_metric              # larger is better (less jerk)
        elif self.metric == "min_time":
            base_fitness =  v_max                    # reward higher speed
        else:
            w_jerk, w_time = 0.7, 0.3                # weights for combined metric
            base_fitness = -(w_jerk*jerk_metric - w_time*v_max)



        return base_fitness #- penalty

    
    def update_particle(self, particle, iteration):
        """Update a particle's velocity and position based on PSO equations.
        
        Args:
            particle: The particle to update
            iteration: Current iteration number
        """
        # Calculate adaptive inertia weight
        w = self.w_max - (self.w_max - self.w_min) * (iteration / self.iterations)
        
        # Generate random coefficients
        r1 = np.random.random(3)  # Cognitive component random factor
        r2 = np.random.random(3)  # Social component random factor
        D = np.random.uniform(-1.0, 1.0, 3)  # Random disturbance
        
        # Calculate new velocity using PSO equation with random disturbance
        cognitive_component = self.c1 * r1 * (particle.p_best - particle.pos)
        social_component = self.c2 * r2 * (self.global_best - particle.pos)
        disturbance = self.alpha * D
        
        particle.vel = w * particle.vel + cognitive_component + social_component + disturbance
        
        # Limit velocity to prevent too large steps
        particle.vel = np.clip(particle.vel, -0.1, 0.1)
        
        # Update position
        particle.pos = particle.pos + particle.vel
        
        # Apply constraints on position
        # Ensure t1 and t2 are in proper order (t1 < t2 < 1.0)
        particle.pos[0] = np.clip(particle.pos[0], self.bounds[0][0], self.bounds[0][1])  # t1
        particle.pos[1] = np.clip(particle.pos[1], max(particle.pos[0] + 0.1, self.bounds[1][0]), self.bounds[1][1])  # t2
        particle.pos[2] = np.clip(particle.pos[2], self.bounds[2][0], self.bounds[2][1])  # v_max
    
    def optimize(self, verbose=True):
        """Run the PSO optimization process.
        
        Args:
            verbose: Whether to print progress updates
            
        Returns:
            tuple: (best_params, fitness_history)
        """
        # Initialize particles
        self.initialize_particles()
        
        # Main optimization loop
        for i in range(self.iterations):
            # Update each particle
            for particle in self.particles:
                # Update particle's position and velocity
                self.update_particle(particle, i)
                
                # Calculate new fitness
                particle.fitness = self.calculate_fitness(particle)
                
                # Update personal best
                if particle.fitness > particle.p_best_fitness:
                    particle.p_best = particle.pos.copy()
                    particle.p_best_fitness = particle.fitness
                    
                    # Update global best if needed
                    if particle.fitness > self.global_best_fitness:
                        self.global_best = particle.pos.copy()
                        self.global_best_fitness = particle.fitness
            
            # Record best fitness of this iteration
            self.fitness_history.append(self.global_best_fitness)
            
            # Print progress every 10 iterations
            if verbose and i % 10 == 0:
                print(f"Iteration {i}: Best Fitness = {self.global_best_fitness}")
                print(f"Best Parameters: t1={self.global_best[0]:.3f}, t2={self.global_best[1]:.3f}, v_max={self.global_best[2]:.3f}")
        
        # Print final results
        if verbose:
            print("\nOptimization Complete!")
            print(f"Best Parameters: t1={self.global_best[0]:.3f}, t2={self.global_best[1]:.3f}, v_max={self.global_best[2]:.3f}")
            print(f"Best Fitness: {self.global_best_fitness}")
        
        return self.global_best, self.fitness_history
    
        
    def plot_fitness_history(self):
        """Plot the fitness history during optimization."""
        plt.figure(figsize=(10, 6))
        plt.plot(self.fitness_history)
        plt.title('Fitness History')
        plt.xlabel('Iteration')
        plt.ylabel('Fitness Value')
        plt.grid(True)
        plt.show()
    
    def get_optimized_parameters(self):
        """Return the optimized trajectory parameters.
        
        Returns:
            dict: Dictionary containing the optimized parameters
                {'t1': float, 't2': float, 'v_max': float}
        """
        if self.global_best is None:
            raise ValueError("Must run optimize() before getting parameters")
            
        t1, t2, v_max = self.global_best
        return {
            't1': t1,
            't2': t2,
            'v_max': v_max
        }
    
    def create_trajectory_config(self):
        """Create a configuration dictionary for the trajectory generator.
        
        Returns:
            dict: Configuration for the MultiAxisTrajectoryGenerator
        """
        if self.global_best is None:
            raise ValueError("Must run optimize() before creating configuration")
            
        return {
            'method': 'trapezoid',
            'mode': 'joint' if self.ndof > 3 else 'task',
            'interval': [0, 1],
            'ndof': self.ndof,
            'start_pos': self.start_pos,
            'final_pos': self.final_pos,
            'optimization_params': self.get_optimized_parameters()
        }


# Integration with the Visualizer class
def add_pso_trajectory_optimization(visualizer):
    """Add PSO trajectory optimization to the existing visualizer.
    
    Args:
        visualizer: The Visualizer instance to modify
    """
    # Add button to the UI
    row_number = len(visualizer.control_frame.grid_slaves()) + 1
    
    visualizer.pso_button = ttk.Button(
        visualizer.control_frame, 
        text="Generate (PSO Optimized)", 
        command=visualizer.generate_pso_optimized_trajectory
    )
    visualizer.pso_button.grid(column=1, row=row_number, columnspan=1, pady=2)
    
    # Add the PSO trajectory generation method
    def generate_pso_optimized_trajectory(self):
        """Generate and follow a PSO-optimized trajectory."""
        print('Generating PSO-optimized trajectory...')
        
        waypoints = self.robot.get_waypoints()
        
        # For joint space trajectory
        if self.robot_type in ['5-dof', 'scara']:
            EE_0 = EndEffector(*waypoints[0][:3], 0, 0, 0)
            EE_f = EndEffector(*waypoints[1][:3], 0, 0, 0)
            
            q0 = np.rad2deg(self.robot.solve_inverse_kinematics(EE_0))
            qf = np.rad2deg(self.robot.solve_inverse_kinematics(EE_f))
            
            # Set up PSO optimizer
            optimizer = PSO_TrajectoryOptimizer(
                start_pos=q0,
                final_pos=qf,
                metric="min_jerk",
                n_particles=30,
                iterations=50,
                ndof=len(q0)
            )
            
            # Run optimization
            optimizer.optimize()
            
            # Get configuration for trajectory generator
            traj_config = optimizer.create_trajectory_config()
            
            # Create trajectory with your existing generator
            traj = MultiAxisTrajectoryGenerator(**traj_config)
            traj_dofs = traj.generate(nsteps=50)
            
            # Execute trajectory
            for i in range(50):
                theta = [dof[0][i] for i, dof in enumerate(traj_dofs)]
                self.update_FK(theta=theta, display_traj=True)
                time.sleep(0.05)
        
        else:  # For task space trajectory
            # Extract start and end points
            start_pos = waypoints[0]
            final_pos = waypoints[1]
            
            # Create PSO optimizer for task space
            optimizer = PSO_TrajectoryOptimizer(
                start_pos=start_pos,
                final_pos=final_pos,
                metric="combined",
                n_particles=30,
                iterations=50,
                ndof=len(start_pos)
            )
            
            # Run optimization
            optimizer.optimize()
            
            # Get configuration
            traj_config = optimizer.create_trajectory_config()
            
            # Create trajectory
            traj = MultiAxisTrajectoryGenerator(**traj_config)
            traj_dofs = traj.generate(nsteps=50)
            
            # Execute trajectory
            for i in range(50):
                pos = [dof[0][i] for i, dof in enumerate(traj_dofs)]
                ee = EndEffector(*pos, 0, -math.pi/2, wraptopi(math.atan2(pos[1], pos[0]) + math.pi))
                self.update_IK(ee, soln=0, numerical=True, display_traj=True)
                time.sleep(0.05)
        
        # Plot trajectory using the MultiAxisTrajectoryGenerator's plot method
        traj.plot()
        
        # Also plot the optimization history
        optimizer.plot_fitness_history()

if __name__ == "__main__":
    start_joint = [0, 0, 0, 0, 0]
    end_joint = [30, 45, -15, 20, 10]
    
    # Create and run optimizer
    optimizer = PSO_TrajectoryOptimizer(
        start_pos=start_joint,
        final_pos=end_joint,
        metric="min_jerk",
        n_particles=50,
        iterations=100,
        ndof=5
    )

    
    print("Trajectory generated successfully!")