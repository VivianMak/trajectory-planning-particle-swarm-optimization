import numpy as np
from typing import List

class Particle():
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, pos=[0, 0, 0], vel=0.0, alpha=1.0):
        """Construct a new Particle
        : the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        alpha: the particle weight (the class does not ensure that particle weights are normalized
        """
        self.alpha = alpha
        self.pos = pos
        self.vel = vel

        self.p_best = None  # Particle Personal Best




class TrajectoryGenerator():
    '''This class tests out the PSO algorithm on different curves
    Attributes List:
        global_best = a shared value between all particles of the global best value??

    
    '''

    def __init__(self, trajectory="Trapezoid"):

        self.n_particles = 100
        self.trajectory = trajectory

        self.iterations = 200

        # Particle attributes
        self.particle_cloud: List[Particle] = []    # Initialize the particle cloud (gaussian distribution around waypoints?)
        self.a_max = 10
        self.v_max = 20

        self.fitness = None
        self.particle_fitness_list = []

        self.w = 1      # Intertia weight
        self.c1 = 1     # Cognitive learning factors
        self.c2 = 1     # Social learning factors

        self.global_best = None

    def initialize_particle_cloud(self):


    def objective_function(self):
        '''Defines the fitness function to evalute the "fitness" of each particle.'''

        # Fitness is recalculated each iteration
        self.fitness = None

        pass

    def update(self):
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

        r1 = np.random.rand(*X.shape)  # Random numbers for cognitive component
        r2 = np.random.rand(*X.shape)  # Random numbers for social component


        old_pos = p.pos
        old_vel = p.vel

        # Calculate new veloctiy
        p.vel =  (self.w * old_vel) + (self.c1 * r1 * (p.p_best - old_pos)) + (self.c2 * r2 * (self.global_best - old_pos))

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

        for value in self.particle_fitness_list:
            if value > self.global_best:
                self.global_best = value


    def normalize_particles(self):
        pass

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