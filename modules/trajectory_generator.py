import numpy as np
import matplotlib.pyplot as plt
from simulator.modules.trajectory_generator import MultiAxisTrajectoryGenerator
from particles import TrajectoryOptimizer #add_pso_trajectory_optimization
from simulator.main_arm import Visualizer
import tkinter as tk
import sys



def main():

    print("Testing PSO Trajectory Optimization")
    print("-----------------------------------")

    optimizer = TrajectoryOptimizer()
    optimizer.test()
    
<<<<<<< HEAD:modules/trajectory_generator.py
    def __init__(self, method="trapezoid",
                 mode="joint",
                 interval=[0,1],
                 ndof=1,
                 start_pos=None,
                 final_pos=None,
                 start_vel=None,
                 final_vel=None,
                 start_acc=None,
                 final_acc=None,
                 ):
        """
        Initialize the trajectory generator with the given configuration.

        Args:
            method (str): Type of trajectory ('linear', 'cubic', 'quintic', 'trapezoid').
            mode (str): 'joint' for joint space, 'task' for task space.
            interval (list): Time interval [start, end] in seconds.
            ndof (int): Number of degrees of freedom.
            start_pos (list): Initial positions.
            final_pos (list): Final positions.
            start_vel (list): Initial velocities (default 0).
            final_vel (list): Final velocities (default 0).
            start_acc (list): Initial accelerations (default 0).
            final_acc (list): Final accelerations (default 0).
        """

        self.T = interval[1]
        self.ndof = ndof
        self.t = None
        
        if mode == "joint":
            self.mode = "Joint Space"
            # self.labels = ['th1', 'th2', 'th3', 'th4', 'th5']
            self.labels = [f'axis{i+1}' for i in range(self.ndof)]
        elif mode == "task":
            self.mode = "Task Space"
            self.labels = ['x', 'y', 'z']
        
        # Assign positions and boundary conditions
        self.start_pos = start_pos
        self.final_pos = final_pos
        self.start_vel = start_vel if start_vel is not None else [0] * self.ndof
        self.final_vel = final_vel if final_vel is not None else [0] * self.ndof
        self.start_acc = start_acc if start_acc is not None else [0] * self.ndof
        self.final_acc = final_acc if final_acc is not None else [0] * self.ndof      

        # Select trajectory generation method
        if method == "linear":
            self.m = LinearInterp(self)
        elif method == "cubic":
            self.m = CubicPolynomial(self)
        elif method == "quintic":
            self.m = QuinticPolynomial(self)
        elif method == "trapezoid":
            self.m = TrapezoidVelocity(self)
    
    def reset_optimization_params(self):
        """Reset any optimization parameters"""
        if hasattr(self, 'optimization_params'):
            self.optimization_params = None
        if hasattr(self, 'm') and hasattr(self.m, 'reset_parameters'):
            self.m.reset_parameters()

=======
    # Define start and end joint positions
    start_joint = [0, 0, 0, 0, 0]
    end_joint = [30, 45, -15, 20, 10]
>>>>>>> ade2681 (worked on vivian_test branch):trajectory.py
    
    print(f"Start position: {start_joint}")
    print(f"End position: {end_joint}")
    
    # Create optimizer with various metrics
    metrics = ["min_jerk", "min_time", "combined"]
    
    for metric in metrics:
        print(f"\nOptimizing with '{metric}' metric:")
        
        # Create and run optimizer
        optimizer = TrajectoryOptimizer(
            metric=metric,
        )

        # Run optimization
        best_params, fitness_history = optimizer.optimization()

        # Get optimization parameters
        params = optimizer.get_optimized_parameters()
        print(f"Optimized parameters: t1={params['t1']:.3f}, t2={params['t2']:.3f}, v_max={params['v_max']:.3f}")
        
        # Create trajectory WITHOUT optimization_params
        traj = MultiAxisTrajectoryGenerator(
            method="trapezoid",
            mode="joint" if len(start_joint) > 3 else "task",
            interval=[0, 1],
            ndof=len(start_joint),
            start_pos=start_joint,
            final_pos=end_joint
        )

        # Manually add the optimization_params to the trajectory object
        traj.optimization_params = params
        
        # Generate trajectory
        traj_data = traj.generate(nsteps=100)
        
        # Plot trajectory 
        traj.plot()
        
        # Plot fitness history
        plt.figure(figsize=(10, 6))
        plt.plot(fitness_history)
        plt.title(f'Fitness History - {metric}')
        plt.xlabel('Iteration')
        plt.ylabel('Fitness Value')
        plt.grid(True)
        plt.show()
        
<<<<<<< HEAD:modules/trajectory_generator.py


class LinearInterp():
    """
    Linear interpolation between start and end positions.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)
        self.solve()


    def _copy_params(self, trajgen):
        self.start_pos = trajgen.start_pos
        self.final_pos = trajgen.final_pos
        self.T = trajgen.T
        self.ndof = trajgen.ndof
        self.X = [None] * self.ndof

    
    def solve(self):
        pass  # Linear interpolation is directly computed in generate()
        

    def generate(self, nsteps=100):
        self.t = np.linspace(0, self.T, nsteps)
        for i in range(self.ndof): # iterate through all DOFs
            q, qd, qdd = [], [], []
            for t in self.t: # iterate through time, t
                q.append((1 - t/self.T)*self.start_pos[i] + (t/self.T)*self.final_pos[i])
                qd.append(self.final_pos[i] - self.start_pos[i])
                qdd.append(0)    
            self.X[i] = [q, qd, qdd]
        return self.X


class CubicPolynomial():
    """
    Cubic interpolation with position and velocity boundary constraints.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)
        self.solve()


    def _copy_params(self, trajgen):
        self.start_pos = trajgen.start_pos
        self.start_vel = trajgen.start_vel
        self.final_pos = trajgen.final_pos
        self.final_vel = trajgen.final_vel
        self.T = trajgen.T
        self.ndof = trajgen.ndof
        self.X = [None] * self.ndof

    
    def solve(self):
        t0, tf = 0, self.T
        self.A = np.array(
                [[1, t0, t0**2, t0**3],
                 [0, 1, 2*t0, 3*t0**2],
                 [1, tf, tf**2, tf**3],
                 [0, 1, 2*tf, 3*tf**2]
                ])
        self.b = np.zeros([4, self.ndof])

        for i in range(self.ndof):
            self.b[:, i] = [self.start_pos[i], self.start_vel[i],
                            self.final_pos[i], self.final_vel[i]]

        self.coeff = np.linalg.solve(self.A, self.b)
        

    def generate(self, nsteps=100):
        self.t = np.linspace(0, self.T, nsteps)

        for i in range(self.ndof): # iterate through all DOFs
            q, qd, qdd = [], [], []
            c = self.coeff[:,i]
            for t in self.t: # iterate through time, t
                q.append(c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3)
                qd.append(c[1] + 2 * c[2] * t + 3 * c[3] * t**2)
                qdd.append(2 * c[2] + 6 * c[3] * t)    
            self.X[i] = [q, qd, qdd]
        return self.X


class QuinticPolynomial():
    """
    Quintic interpolation with position, velocity, and acceleration constraints.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)
        self.solve()

    def _copy_params(self, trajgen):
        self.start_pos = trajgen.start_pos
        self.start_vel = trajgen.start_vel
        self.start_acc = trajgen.start_acc
        self.final_pos = trajgen.final_pos
        self.final_vel = trajgen.final_vel
        self.final_acc = trajgen.final_acc
        self.T = trajgen.T
        self.ndof = trajgen.ndof
        self.X = [None] * self.ndof
    
    def solve(self):
        t0, tf = 0, self.T
        self.A = np.array(
                [[1, t0, t0**2, t0**3, t0**4, t0**5],
                [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3],
                ])
        
        self.b = np.zeros([6, self.ndof])

        for i in range(self.ndof):
            self.b[:, i] = [self.start_pos[i], self.start_vel[i], self.start_acc[i],
                            self.final_pos[i], self.final_vel[i], self.final_acc[i]]

        self.coeff = np.linalg.solve(self.A, self.b)

    def generate(self, nsteps=100):
        self.t = np.linspace(0, self.T, nsteps)

        for i in range(self.ndof): # iterate through all DOFs
            q, qd, qdd = [], [], []
            c = self.coeff[:,i]
            for t in self.t: # iterate through time, t
                q.append(c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3 + c[4] * t**4 + c[5] * t**5)
                qd.append(c[1] + 2 * c[2] * t + 3 * c[3] * t**2 + 4 * c[4] * t**3 + 5 * c[5] * t**4)
                qdd.append(2 * c[2] + 6 * c[3] * t + 12 * c[4] * t**2 + 20 * c[5] * t**3)    
            self.X[i] = [q, qd, qdd]
        return self.X
    


class TrapezoidVelocity():
    """
    Trapezoidal velocity profile generator for constant acceleration/deceleration phases.
    """

    def __init__(self, trajgen):
        self._copy_params(trajgen)
        self.solve()

    def _copy_params(self, trajgen):
        self.start_pos = trajgen.start_pos
        self.start_vel = trajgen.start_vel
        self.start_acc = trajgen.start_acc
        self.final_pos = trajgen.final_pos
        self.final_vel = trajgen.final_vel
        self.final_acc = trajgen.final_acc
        self.T = trajgen.T
        self.ndof = trajgen.ndof
        self.X = [None] * self.ndof


    def solve(self):
<<<<<<< HEAD:simulator/modules/trajectory_generator.py
        t0, tf = 0, self.T
=======
        """
        Compute parameters for each DOF:
        - t_accel: duration of acceleration phase
        - t_cruise: duration of constant-speed cruise phase
        - t_decel: end time of deceleration (equal to total T)
        - v_cruise: constant cruise speed to satisfy boundary conditions
        - accel/decel magnitudes based on v_cruise
        """
        if hasattr(self, 'optimization_params') and self.optimization_params is not None:
            print("In solve(): Using optimization params:", self.optimization_params)
            # Use optimized parameters
            for i in range(self.ndof):
                p = self.optimization_params
                t_accel = p['t1'] * self.T
                t_decel = p['t2'] * self.T
                t_cruise = self.T - t_accel - t_decel
                
                v_cruise = p['v_max']
                accel = v_cruise / t_accel if t_accel > 0 else 0
                decel = -v_cruise / t_decel if t_decel > 0 else 0
                
                self.params[i] = {
                    'accel': accel,
                    'decel': decel,
                    'v_cruise': v_cruise,
                    't_accel': t_accel,
                    't_cruise': t_accel + t_cruise,
                    't_decel': self.T
                }
        else:
            print("In solve(): Using default ratios:", self.accel_time_ratio, self.decel_time_ratio)
            for i in range(self.ndof):
                # Phase durations
                t_accel = self.T * self.accel_time_ratio   # time accelerating
                t_decel = self.T * self.decel_time_ratio   # time decelerating
                t_cruise = self.T - t_accel - t_decel      # time at constant speed

                # Displacement to cover
                total_distance = self.final_pos[i] - self.start_pos[i]

                # Solve for cruise velocity using area under velocity-time graph:
                # distance = v_cruise*t_cruise + 0.5*v_cruise*t_accel + 0.5*v_cruise*t_decel
                v_cruise = total_distance / (t_cruise + 0.5*t_accel + 0.5*t_decel)

                # Constant acceleration: a = v_cruise / t_accel
                accel = v_cruise / t_accel
                # Constant deceleration: decel = -v_cruise / t_decel
                decel = -v_cruise / t_decel

                self.params[i] = {
                    'accel': accel,
                    'decel': decel,
                    'v_cruise': v_cruise,
                    't_accel': t_accel,                   # end of accel phase
                    't_cruise': t_accel + t_cruise,       # end of cruise phase
                    't_decel': self.T                     # end of decel phase (total)
                }
>>>>>>> 4581370 (Add pso_integration):modules/trajectory_generator.py

    
    
    def generate(self, nsteps=100):
<<<<<<< HEAD:simulator/modules/trajectory_generator.py
        self.t = np.linspace(0, self.T, nsteps)

        for i in range(self.ndof): # iterate through all DOFs
            q, qd, qdd = [], [], []
            c = self.coeff[:,i]
            for t in self.t: # iterate through time, t
                q.append()
                qd.append()
                qdd.append()  
            self.X[i] = [q, qd, qdd]
        return self.X
=======
            """
            Sample trajectory at nsteps between t=0 and t=T.
            Returns list per DOF: [positions, velocities, accelerations].
            """
            print("\nGenerating trajectory:")
            if hasattr(self, 'optimization_params') and self.optimization_params is not None:
                    print("Using optimization params:", self.optimization_params)
            else:
                    print("Using default ratios:", self.accel_time_ratio, self.decel_time_ratio)
            self.solve()
            print("Resulting parameters for joint 1:")
            print("t_accel:", self.params[0]['t_accel'], "v_cruise:", self.params[0]['v_cruise'])
            
            self.t = np.linspace(0, self.T, nsteps)

            for i in range(self.ndof):
                p = self.params[i]
                t_accel = p['t_accel']
                t_cruise = p['t_cruise']

                # Precompute end-of-acceleration and end-of-cruise positions
                q_accel_end = (
                    self.start_pos[i]
                    + self.start_vel[i]*t_accel
                    + 0.5*p['accel']*t_accel**2
                )
                cruise_duration = t_cruise - t_accel
                q_cruise_end = q_accel_end + p['v_cruise']*cruise_duration

                q, qd, qdd = [], [], []
                for t in self.t:
                    # Phase 1: Acceleration
                    if t < t_accel:
                        pos = (
                            self.start_pos[i]
                            + self.start_vel[i]*t
                            + 0.5*p['accel']*t**2
                        )
                        vel = self.start_vel[i] + p['accel']*t
                        acc = p['accel']

                    # Phase 2: Cruise (constant velocity)
                    elif t < t_cruise:
                        t_since_accel = t - t_accel
                        pos = q_accel_end + p['v_cruise']*t_since_accel
                        vel = p['v_cruise']
                        acc = 0

                    # Phase 3: Deceleration
                    else:
                        t_since_decel = t - t_cruise
                        pos = (
                            q_cruise_end
                            + p['v_cruise']*t_since_decel
                            + 0.5*p['decel']*t_since_decel**2
                        )
                        vel = p['v_cruise'] + p['decel']*t_since_decel
                        acc = p['decel']

                    q.append(pos)
                    qd.append(vel)
                    qdd.append(acc)

                self.X[i] = [q, qd, qdd]
            return self.X
    def reset_parameters(self):
        """Reset any optimization parameters to use default values"""
        if hasattr(self, 'optimization_params'):
            self.optimization_params = None
        # Re-solve with default parameters
        self.solve()
>>>>>>> 4581370 (Add pso_integration):modules/trajectory_generator.py
=======
        print(f"Trajectory with '{metric}' metric generated successfully!")
>>>>>>> ade2681 (worked on vivian_test branch):trajectory.py
