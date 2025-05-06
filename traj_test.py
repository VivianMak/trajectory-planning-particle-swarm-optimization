import numpy as np
import matplotlib.pyplot as plt
from modules.trajectory_generator import MultiAxisTrajectoryGenerator
from particles_test import PSO_TrajectoryOptimizer
from main_arm import Visualizer
import tkinter as tk
import sys

def main():
    print("Testing PSO Trajectory Optimization")
    print("-----------------------------------")
    
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
        optimizer = PSO_TrajectoryOptimizer(
            start_pos=start_joint,
            final_pos=end_joint,
            metric=metric,
            n_particles=30,  # Lower for testing
            iterations=50,   # Lower for testing
            ndof=len(start_joint)
        )
        
        # Run optimization
        best_params, fitness_history = optimizer.optimize()
        
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

def compare_with_default():
    """Compare PSO-optimized trajectory with default trapezoidal trajectory"""
    print("\nComparing PSO-optimized vs. Default Trajectory")
    print("-------------------------------------------")
    
    # Define start and end joint positions
    start_joint = [0, 0, 0, 0, 0]
    end_joint = [30, 45, -15, 20, 10]
    
    # Create default trajectory
    default_traj = MultiAxisTrajectoryGenerator(
        method="trapezoid",
        mode="joint",
        interval=[0, 1],
        ndof=len(start_joint),
        start_pos=start_joint,
        final_pos=end_joint
    )
    
    # Generate default trajectory
    default_data = default_traj.generate(nsteps=100)
    
    # Create and run optimizer
    optimizer = PSO_TrajectoryOptimizer(
        start_pos=start_joint,
        final_pos=end_joint,
        metric="min_time",
        n_particles=100,
        iterations=100,
        ndof=len(start_joint)
    )
    
    # Run optimization
    best_params, fitness_history = optimizer.optimize()
    
    # Create optimized trajectory
    params = optimizer.get_optimized_parameters()

    traj = MultiAxisTrajectoryGenerator(
    method="trapezoid",
    mode="joint" if len(start_joint) > 3 else "task",
    interval=[0, 1],
    ndof=len(start_joint),
    start_pos=start_joint,
    final_pos=end_joint
    )

    traj.optimization_params = params
    
    # Generate optimized trajectory
    optimized_data = traj.generate(nsteps=100)
    
    # Compare trajectories (position, velocity, acceleration)
    time = np.linspace(0, 1, 100)
    
    # Create figure to compare
    fig, axs = plt.subplots(3, 1, figsize=(12, 15))
    
    # Position comparison (first joint)
    axs[0].plot(time, default_data[0][0], 'r--', label='Default Position')
    axs[0].plot(time, optimized_data[0][0], 'b-', label='Optimized Position')
    axs[0].set_title('Position Comparison (Joint 1)')
    axs[0].set_ylabel('Position')
    axs[0].grid(True)
    axs[0].legend()
    
    # Velocity comparison
    axs[1].plot(time, default_data[0][1], 'r--', label='Default Velocity')
    axs[1].plot(time, optimized_data[0][1], 'b-', label='Optimized Velocity')
    axs[1].set_title('Velocity Comparison (Joint 1)')
    axs[1].set_ylabel('Velocity')
    axs[1].grid(True)
    axs[1].legend()
    
    # Acceleration comparison
    axs[2].plot(time, default_data[0][2], 'r--', label='Default Acceleration')
    axs[2].plot(time, optimized_data[0][2], 'b-', label='Optimized Acceleration')
    axs[2].set_title('Acceleration Comparison (Joint 1)')
    axs[2].set_xlabel('Time')
    axs[2].set_ylabel('Acceleration')
    axs[2].grid(True)
    axs[2].legend()
    
    plt.tight_layout()
    plt.show()
    
    print("Comparison complete!")

def visualize_trajectory():

    print("Testing PSO Trajectory Optimization with Visualizer")
    print("------------------------------------------------")
    
    # Create root window
    root = tk.Tk()
    
    # Create command-line arguments (modify as needed for your visualizer)
    class Args:
        robot_type = '5-dof'  # or whatever robot type you want to test
    
    args = Args()
    
    # Create the visualizer
    app = Visualizer(root, args)
    
    # Add PSO trajectory optimization functionality
    add_pso_trajectory_optimization(app)
    
    # Run the Tkinter main loop
    root.mainloop()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        test_type = sys.argv[1]
        if test_type == "basic":
            main()
        elif test_type == "compare":
            compare_with_default()
        elif test_type == "visualize":
            visualize_trajectory()
        else:
            print("Invalid test type. Use 'basic', 'compare', or 'visualize'.")
    else:
        # Run all tests by default
        main()
        compare_with_default()
        visualize_trajectory()
