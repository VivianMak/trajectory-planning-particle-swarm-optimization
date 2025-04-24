import numpy as np
from numpy import random
from typing import List

import matplotlib.pyplot as plt

from mp_code.helper_fcns.utils import EndEffector, rotm_to_euler


class Particle():
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, pos=[0, 0, 0], vel=0.0, alpha1=1.0):
        """Construct a new Particle
        pos: the x-coordinate of the hypothesis relative to the map frame
        vel: the y-coordinate of the hypothesis relative ot the map frame
        alpha: the particle weight (the class does not ensure that particle weights are normalized
        """
        self.alpha1 = alpha1
        self.pos = pos
        self.vel = vel

        self.p_best = None  # Particle Personal Best




class TrajectoryOptimizer():
    '''This class tests out the PSO algorithm on different curves
    Attributes List:
        global_best = a shared value between all particles of the global best value??

    
    '''

    def __init__(self, trajectory="Trapezoid"):

        self.n_particles = 100
        self.trajectory = trajectory

        self.iterations = 200
        self.current_iteration = 0

        # Particle attributes
        self.particle_cloud: List[Particle] = []    # Initialize the particle cloud (gaussian distribution around waypoints?)
        self.a_max = 10
        self.v_max = 20

        self.fitness = None
        self.particle_fitness_list = []

        self.alpha = 1      # Random Distrubance Coefficient
        self.w_min = 1      # Intertia weight
        self.w_max = 2
        self.c1 = 1     # Cognitive learning factors
        self.c2 = 1     # Social learning factors

        self.global_best = None

    def initialize_particle_cloud(self):#, EE: EndEffector):
        """Iniitlize a set of particles"""


        EE_pose = [0.5, 0.5, 0.5]

        particles_dict = {"x_distr": [], "y_distr": [], "z_distr": []}

        # Create and add n particles to the particle cloud list
        for i, key in enumerate(particles_dict.keys()):

            particles_dict[key] = random.normal(
                loc=EE_pose[i], scale=1, size=100
            )
        
        print(particles_dict)

        for i in range(100):

            # Initializes each particle with value in the distribution
            p = Particle(
                pos = [
                    particles_dict["x_distr"][i],
                    particles_dict["y_distr"][i],
                    particles_dict["z_distr"][i],
                ],
                vel = 0,
                alpha1 = 0.1,
            )
            
            # Sets initial value to personal best
            p.p_best = [
                particles_dict["x_distr"][i],
                particles_dict["y_distr"][i],
                particles_dict["z_distr"][i]
            ]
            
            # Add this particle to the cloud
            self.particle_cloud.append(p)



    def objective_function(self):
        '''Defines the fitness function to evalute the "fitness" of each particle.'''

        # Fitness is recalculated each iteration
        self.fitness = None

        pass

    def update(self, p: Particle):
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

        w = self.w_max - (self.w_max - self.w_min) * (self.current_iteration / self.iterations)


        old_pos = p.pos
        old_vel = p.vel

        # Calculate new veloctiy
        p.vel =  (w * old_vel) + (self.c1 * r1 * (p.p_best - old_pos)) + (self.c2 * r2 * (self.global_best - old_pos))

        p.pos = old_pos + p.vel

        pass


    def optimization(self):
        '''Runs the particle swarm optimization.'''
        # for i in range(self.iterations):
        # based on movement, determine indv fitness
        for p in self.particle_cloud:
            self.objective_function()

        
            # update particle individual optimal solution
            # when the fitness exceeds indv optimal solution, update pos and velocity
            if self.fitness > p.p_best:
                p.p_best = self.fitness


            # Add each particle's fitness value to a list to compare best solutions
            self.particle_fitness_list.append(self.fitness)

            # Update each particle's position, velocity, personal best
            self.update(p)

        for value in self.particle_fitness_list:
            if value > self.global_best:
                self.global_best = value


    def normalize_particles(self):
        pass

    def visualizer(self):#, EE: EndEffector):
        """Plotting the particles and end effector pose"""
        x_vals = [p.pos[0] for p in self.particle_cloud]
        y_vals = [p.pos[1] for p in self.particle_cloud]
        z_vals = [p.pos[2] for p in self.particle_cloud]

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.scatter(x_vals, y_vals, z_vals, c='blue', s=20, label='Particles')
        ax.scatter(0.5, 0.5, 0.5, c='red', s=100, marker='*', label='End Effector Pose')


        plt.title("Gaussian Particle Distribution around End-Effector Pose")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()


    '''         
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
                choices: the values to sample from represented as a list
                probabilities: the probability of selecting each element in choices represented as a list
                n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples
    '''