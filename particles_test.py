import numpy as np
import matplotlib.pyplot as plt
from trajectory import MultiAxisTrajectoryGenerator
import time
from helper_fcns.utils import EndEffector
import math
from helper_fcns.utils import wraptopi
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
        self.alpha = 0.5      # Used for adaptive learning factors
        self.beta = 0.5       # Used for adaptive learning factors  
        
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
        """Update a particle's velocity and position based on PSO equations with adaptive parameters.
        
        Args:
            particle: The particle to update
            iteration: Current iteration number
        """
        # Calculate fitness statistics for adaptive parameters
        if len(self.fitness_history) > 0:
            f_min = min([p.fitness for p in self.particles if p.fitness is not None])
            f_sum = sum([p.fitness for p in self.particles if p.fitness is not None])
            f_count = sum([1 for p in self.particles if p.fitness is not None])
            f_avg = f_sum / f_count if f_count > 0 else 0
        else:
            f_min = 0
            f_avg = 0
        
        # Calculate adaptive inertia weight based on fitness
        if f_avg != f_min:
            w = self.w_max - (self.w_max - self.w_min) * ((particle.fitness - f_min) / (f_avg - f_min))
        else:
            w = self.w_max
        
        # Calculate adaptive learning factors
        if f_avg != f_min:
            # Cognitive learning factor (personal best)
            c1 = self.alpha * ((particle.fitness - f_min) / (f_avg - f_min)) + self.beta
            # Social learning factor (global best)
            c2 = self.alpha * (1 - (particle.fitness - f_min) / (f_avg - f_min)) + self.beta
        else:
            c1 = self.alpha + self.beta
            c2 = self.alpha + self.beta
        
        # Generate random coefficients
        r1 = np.random.random(3)  # Cognitive component random factor
        r2 = np.random.random(3)  # Social component random factor
        D = np.random.uniform(-1.0, 1.0, 3)  # Random disturbance
        
        # Save old values
        old_pos = particle.pos
        old_vel = particle.vel
        
        # Calculate new velocity using adaptive PSO equation with random disturbance
        cognitive_component = c1 * r1 * (particle.p_best - old_pos)
        social_component = c2 * r2 * (self.global_best - old_pos)
        random_disturbance = self.alpha * D
        
        particle.vel = w * old_vel + cognitive_component + social_component + random_disturbance
        
        # Limit velocity to prevent too large steps
        particle.vel = np.clip(particle.vel, -0.1, 0.1)
        
        # Update position
        particle.pos = old_pos + particle.vel
        
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