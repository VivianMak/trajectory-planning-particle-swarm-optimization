import numpy as np
import matplotlib.pyplot as plt
from modules.trajectory_generator import MultiAxisTrajectoryGenerator
from particles import TrajectoryOptimizer #add_pso_trajectory_optimization
from main_arm import Visualizer
import tkinter as tk
import sys



def main():

    print("Testing PSO Trajectory Optimization")
    print("-----------------------------------")

    # optimizer = TrajectoryOptimizer()
    # optimizer.test()
    
    # Define start and end joint positions
    start_joint = [0, 0, 0, 0, 0]
    end_joint = [30, 45, -15, 20, 10]
    
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
        
        print(f"Trajectory with '{metric}' metric generated successfully!")