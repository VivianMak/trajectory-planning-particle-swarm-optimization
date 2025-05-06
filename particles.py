import numpy as np
from numpy import random
from typing import List

import matplotlib.pyplot as plt

from helper_fcns.utils import EndEffector, rotm_to_euler


class Particle():
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, pos=[0, 0, 0], vel=0.0):
        """Construct a new Particle
        pos: the x-coordinate of the hypothesis relative to the map frame
        vel: the y-coordinate of the hypothesis relative ot the map frame
        alpha: the particle weight (the class does not ensure that particle weights are normalized
        """
        self.pos = pos
        self.vel = vel

        self.p_best = None  # Particle Personal Best

        self.particle_fitness_history = []




class TrajectoryOptimizer():
    '''This class tests out the PSO algorithm on different curves
    Attributes List:
        global_best = a shared value between all particles of the global best value??

    
    '''

    def __init__(self, metric="min_jerk"):

        self.n_particles = 50
        self.iterations = 100
        self.metric = metric

        # Particle attributes
        self.particle_cloud: List[Particle] = []    # Initialize the particle cloud (gaussian distribution around waypoints?)
        self.a_max = 10
        self.v_max = 20

        self.fitness = None
        self.fitness_history = []

        self.alpha = 1      # Random Distrubance Coefficient
        self.w_min = 1      # Intertia weight
        self.w_max = 10
        # self.c1 = 1     # Cognitive learning factors
        # self.c2 = 1     # Social learning factors
        self.alpha = np.random.rand([0.0, 1.0])
        self.beta = np.random.rand([0.0, 1.0])

        self.global_best = None

        self.n_dof = 5

    def initialize_particle_cloud(self):
        """Iniitlize a set of particles"""


        curve_param = [0.5, 0.5, 0.5]

        particles_dict = {"t1_distr": [], "t2_distr": [], "v_max_distr": []}

        # Create and add n particles to the particle cloud list
        for i, key in enumerate(particles_dict.keys()):

            particles_dict[key] = random.normal(
                loc=curve_param[i], scale=1, size=100
            )
        
        print(particles_dict)

        for i in range(100):

            # Initializes each particle with value in the distribution
            p = Particle(
                pos = [
                    particles_dict["t1_distr"][i],
                    particles_dict["t2_distr"][i],
                    particles_dict["v_max_distr"][i],
                ],
                vel = 0,
            )
            
            # Sets initial value to personal best
            p.p_best = p.pos
            
            # Add this particle to the cloud
            self.particle_cloud.append(p)



    def objective_function(self):
        '''Defines the fitness function to evalute the "fitness" of each particle.'''

        # Fitness is recalculated each iteration
        self.fitness = None

        if self.metric == "min_jerk":
            pass
        elif self.metric == "min_time":
            pass

        pass

    def update(self, p: Particle, iteration):
        '''Updates each particle with the position and velocity.
        
        Parameters:
        - X: Current positions (shape: [num_particles, dim])
        - V: Current velocities (shape: [num_particles, dim])
        - pb: Personal best positions (shape: [num_particles, dim])
        - gb: Global best position (shape: [dim])
        - w: Inertia weight (scalar)
        - c1: Cognitive learning factor (scalar)
        - c2: Social learning factor (scalar)

        Returns:
        - X_new: Updated positions
        - V_new: Updated velocities
   
        '''

        r1 = np.random.rand([0.0, 1.0])  # Random numbers for cognitive component
        r2 = np.random.rand([0.0, 1.0])  # Random numbers for social component
        D = np.random.rand([-1.0, 1.0])

        w = self.w_max - (self.w_max - self.w_min) * ((self.fitness - f_min) / (f_avg - f_min))

        f_min = min(p.particle_fitness_history)
        f_avg = sum(p.particle_fitness_history) / len(p.particle_fitness_history)
        # Cognitive learning factors
        c1 = self.alpha * ((self.fitness - f_min) / (f_avg - f_min)) + self.beta
        # Social learning factors
        c2 = self.alpha * (1 - (self.fitness - f_min) / (f_avg - f_min)) + self.beta 

        old_pos = p.pos
        old_vel = p.vel

        # Calculate new veloctiy
        p.vel =  (w * old_vel) + (c1 * r1 * (p.p_best - old_pos)) + (c2 * r2 * (self.global_best - old_pos)) + self.alpha + D

        p.pos = old_pos + p.vel
    
    def test(self):
        print("HELLO???")

    def optimization(self, verbose=True):
        '''Runs the particle swarm optimization.'''
        
        # Initialize particles
        self.initialize_particle_cloud()
        print(f"PARTICLE CLOUD LSIT IS INITIALIZED TO {len(self.particle_cloud)}")

        # Optimization Loop
        for i in range(self.iterations):
            for p in self.particle_cloud:
                self.objective_function()

                # update particle individual optimal solution
                # when the fitness exceeds indv optimal solution, update pos and velocity
                if self.fitness > p.p_best:
                    p.p_best = self.fitness

                if p.p_best > self.global_best:
                    self.global_best = p.p_best

                # Update each particle's position, velocity, personal best
                self.update(p, i)

                p.particle_fitness_history.apped(self.fitness)

            # Add global best of eahc iteration
            self.fitness_history.append(self.global_best)

            ##################################
            # Print progress every 10 iterations
            if verbose and i % 10 == 0:
                print(f"Iteration {i}: Best Fitness = {self.global_best}")
                print(f"Best Parameters: t1={self.global_best[0]:.3f}, t2={self.global_best[1]:.3f}, v_max={self.global_best[2]:.3f}")
        
        # Print final results
        if verbose:
            print("\nOptimization Complete!")
            print(f"Best Parameters: t1={self.global_best[0]:.3f}, t2={self.global_best[1]:.3f}, v_max={self.global_best[2]:.3f}")
            print(f"Best Fitness: {self.global_best}")
        
        return self.global_best, self.fitness_history

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


    # def visualizer(self):#, EE: EndEffector):
    #     """Plotting the particles and end effector pose"""
    #     x_vals = [p.pos[0] for p in self.particle_cloud]
    #     y_vals = [p.pos[1] for p in self.particle_cloud]
    #     z_vals = [p.pos[2] for p in self.particle_cloud]

    #     fig = plt.figure()
    #     ax = fig.add_subplot(projection='3d')

    #     ax.scatter(x_vals, y_vals, z_vals, c='blue', s=20, label='Particles')
    #     ax.scatter(0.5, 0.5, 0.5, c='red', s=100, marker='*', label='End Effector Pose')


    #     plt.title("Gaussian Particle Distribution around End-Effector Pose")
    #     plt.xlabel("X")
    #     plt.ylabel("Y")
    #     plt.axis("equal")
    #     plt.grid(True)
    #     plt.legend()
    #     plt.show()


<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> ade2681 (worked on vivian_test branch)
=======

>>>>>>> 4581370 (Add pso_integration)
if __name__ == "__main__":
    start_joint = [0, 0, 0, 0, 0]
    end_joint = [30, 45, -15, 20, 10]
    
    # Create and run optimizer
    optimizer = TrajectoryOptimizer(
        metric="min_jerk"
    )

    
<<<<<<< HEAD
    print("Trajectory generated successfully!")
=======
    print("Trajectory generated successfully!")
>>>>>>> ade2681 (worked on vivian_test branch)
