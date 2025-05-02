import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import ttk, messagebox
import time
from helper_fcns.utils import EndEffector

def add_pso_trajectory_optimization(visualizer):
    """
    Adds PSO trajectory optimization functionality to the Visualizer class.
    This function is called after the Visualizer is fully initialized to avoid circular imports.
    
    Args:
        visualizer: Instance of the Visualizer class to add PSO functionality to
    """
    from particles_test import PSO_TrajectoryOptimizer
    from modules.trajectory_generator import MultiAxisTrajectoryGenerator
    
    visualizer.PSO_TrajectoryOptimizer = PSO_TrajectoryOptimizer
    visualizer.MultiAxisTrajectoryGenerator = MultiAxisTrajectoryGenerator
    
    row_number = _get_last_row(visualizer.control_frame) + 2
    
    visualizer.pso_entry_title = ttk.Label(visualizer.control_frame, text="PSO Trajectory Optimization:", font=("Arial", 13, "bold"))
    visualizer.pso_entry_title.grid(column=0, row=row_number, columnspan=2, pady=(10, 10))
    row_number += 1
    
    visualizer.metric_label = ttk.Label(visualizer.control_frame, text="Optimization Metric:")
    visualizer.metric_label.grid(column=0, row=row_number, sticky=tk.W)
    
    visualizer.metric_var = tk.StringVar(value="min_jerk")
    visualizer.metric_combo = ttk.Combobox(visualizer.control_frame, textvariable=visualizer.metric_var, 
                                    values=["min_jerk", "min_time", "combined"])
    visualizer.metric_combo.grid(column=1, row=row_number)
    row_number += 1
    
    visualizer.particles_label = ttk.Label(visualizer.control_frame, text="Number of Particles:")
    visualizer.particles_label.grid(column=0, row=row_number, sticky=tk.W)
    
    visualizer.particles_entry = ttk.Entry(visualizer.control_frame)
    visualizer.particles_entry.insert(0, "30")
    visualizer.particles_entry.grid(column=1, row=row_number)
    row_number += 1
    
    visualizer.iterations_label = ttk.Label(visualizer.control_frame, text="Number of Iterations:")
    visualizer.iterations_label.grid(column=0, row=row_number, sticky=tk.W)
    
    visualizer.iterations_entry = ttk.Entry(visualizer.control_frame)
    visualizer.iterations_entry.insert(0, "50")
    visualizer.iterations_entry.grid(column=1, row=row_number)
    row_number += 1
    
    visualizer.optimize_button = ttk.Button(visualizer.control_frame, text="Optimize & Execute", 
                                      command=lambda: run_pso_optimization(visualizer))
    visualizer.optimize_button.grid(column=0, row=row_number, columnspan=2, pady=5)
    row_number += 1
    
    visualizer.compare_button = ttk.Button(visualizer.control_frame, text="Compare with Default", 
                                    command=lambda: compare_with_default(visualizer))
    visualizer.compare_button.grid(column=0, row=row_number, columnspan=2, pady=5)


def _get_last_row(frame):
    """Helper method to find the last used row in the control frame"""
    return max([int(child.grid_info()['row']) for child in frame.winfo_children() if 'row' in child.grid_info()])


def run_pso_optimization(visualizer):
    """
    Runs the PSO optimization algorithm and executes the optimized trajectory.
    
    Args:
        visualizer: The Visualizer instance
    """
    if not hasattr(visualizer.robot, 'waypoint_x') or len(visualizer.robot.waypoint_x) < 2:
        messagebox.showerror("Error", "Please upload waypoints first!")
        return
    
    waypoints = visualizer.robot.get_waypoints()
    
    start_ee = EndEffector(*waypoints[0], 0, 0, 0)
    end_ee = EndEffector(*waypoints[1], 0, 0, 0)
    
    try:
        start_joint = np.rad2deg(visualizer.robot.solve_inverse_kinematics(start_ee))
        end_joint = np.rad2deg(visualizer.robot.solve_inverse_kinematics(end_ee))
    except Exception as e:
        messagebox.showerror("IK Error", f"Could not solve IK for waypoints: {str(e)}")
        return
    
    try:
        metric = visualizer.metric_var.get()
        n_particles = int(visualizer.particles_entry.get())
        iterations = int(visualizer.iterations_entry.get())
    except ValueError:
        messagebox.showerror("Input Error", "Please enter valid numbers for particles and iterations")
        return
    
    progress_window = tk.Toplevel(visualizer.root)
    progress_window.title("Optimization Progress")
    progress_label = ttk.Label(progress_window, text="PSO optimization in progress...\nThis may take a moment.")
    progress_label.pack(padx=20, pady=20)
    progress_window.update()
    
    try:
        optimizer = visualizer.PSO_TrajectoryOptimizer(
            start_pos=start_joint,
            final_pos=end_joint,
            metric=metric,
            n_particles=n_particles,
            iterations=iterations,
            ndof=len(start_joint)
        )
        
        best_params, fitness_history = optimizer.optimize()
        
        progress_window.destroy()
        
        params = optimizer.get_optimized_parameters()
        print(f"Optimized parameters: t1={params['t1']:.3f}, t2={params['t2']:.3f}, v_max={params['v_max']:.3f}")
        
        traj = visualizer.MultiAxisTrajectoryGenerator(
            method="trapezoid",
            mode="joint",
            interval=[0, 1],
            ndof=len(start_joint),
            start_pos=start_joint,
            final_pos=end_joint
        )
        
        traj.optimization_params = params
        
        traj_data = traj.generate(nsteps=100)
        
        execute_trajectory(visualizer, traj_data)
        
        plt.figure(figsize=(10, 6))
        plt.plot(fitness_history)
        plt.title(f'Fitness History - {metric}')
        plt.xlabel('Iteration')
        plt.ylabel('Fitness Value')
        plt.grid(True)
        plt.show()
        
    except Exception as e:
        progress_window.destroy()
        messagebox.showerror("Optimization Error", f"Error during optimization: {str(e)}")


def execute_trajectory(visualizer, traj_data, time_per_step=0.05):
    """
    Executes a trajectory by moving the robot through the generated path.
    
    Args:
        visualizer: The Visualizer instance
        traj_data: The trajectory data from the trajectory generator
        time_per_step: Time to wait between steps (controls execution speed)
    """
    visualizer.robot.reset_ee_trajectory()
    
    nsteps = len(traj_data[0][0])
    
    for i in range(nsteps):
        theta = [dof[0][i] for dof in traj_data]
        
        visualizer.update_FK(theta=theta, display_traj=True)
        
        visualizer.root.update()
            
    print("Trajectory execution complete!")


def compare_with_default(visualizer):
    """
    Compares the PSO-optimized trajectory with the default trajectory using normalized plots.
    """
    # First make sure we have waypoints to work with
    if not hasattr(visualizer.robot, 'waypoint_x') or len(visualizer.robot.waypoint_x) < 2:
        messagebox.showerror("Error", "Please upload waypoints first!")
        return
    
    waypoints = visualizer.robot.get_waypoints()
    
    # Solve IK for start and end points to get joint configurations
    start_ee = EndEffector(*waypoints[0], 0, 0, 0)
    end_ee = EndEffector(*waypoints[1], 0, 0, 0)
    
    try:
        start_joint = np.rad2deg(visualizer.robot.solve_inverse_kinematics(start_ee))
        end_joint = np.rad2deg(visualizer.robot.solve_inverse_kinematics(end_ee))
    except Exception as e:
        messagebox.showerror("IK Error", f"Could not solve IK for waypoints: {str(e)}")
        return
    
    # Get PSO parameters from UI
    try:
        metric = visualizer.metric_var.get()
        n_particles = int(visualizer.particles_entry.get())
        iterations = int(visualizer.iterations_entry.get())
    except ValueError:
        messagebox.showerror("Input Error", "Please enter valid numbers for particles and iterations")
        return
    
    # Show a progress message
    progress_window = tk.Toplevel(visualizer.root)
    progress_window.title("Comparison Progress")
    progress_label = ttk.Label(progress_window, text="Generating trajectories for comparison...\nThis may take a moment.")
    progress_label.pack(padx=20, pady=20)
    progress_window.update()
    
    try:
        # Create default trajectory
        default_traj = visualizer.MultiAxisTrajectoryGenerator(
            method="trapezoid",
            mode="joint",
            interval=[0, 1],
            ndof=len(start_joint),
            start_pos=start_joint,
            final_pos=end_joint
        )
        
        # Reset any optimization parameters
        if hasattr(default_traj, 'reset_optimization_params'):
            default_traj.reset_optimization_params()
        elif hasattr(default_traj, 'm') and hasattr(default_traj.m, 'optimization_params'):
            default_traj.m.optimization_params = None

        # Generate default trajectory
        default_data = default_traj.generate(nsteps=100)
        
        # Create and run optimizer
        optimizer = visualizer.PSO_TrajectoryOptimizer(
            start_pos=start_joint,
            final_pos=end_joint,
            metric=metric,
            n_particles=n_particles,
            iterations=iterations,
            ndof=len(start_joint)
        )
        
        # Run optimization
        best_params, fitness_history = optimizer.optimize()
        
        # Get optimization parameters
        params = optimizer.get_optimized_parameters()
        
        # Create optimized trajectory
        traj = visualizer.MultiAxisTrajectoryGenerator(
            method="trapezoid",
            mode="joint",
            interval=[0, 1],
            ndof=len(start_joint),
            start_pos=start_joint,
            final_pos=end_joint
        )
        
        # Set optimization parameters
        traj.optimization_params = params
        if hasattr(traj, 'm'):
            traj.m.optimization_params = params
        
        # Generate optimized trajectory
        optimized_data = traj.generate(nsteps=100)
        
        # Close progress window
        progress_window.destroy()
        
        # Normalization function
        def normalize_data(data):
            data_min = min(data)
            data_max = max(data)
            if data_max == data_min:
                return [0.5] * len(data)  # Handle constant data
            return [(x - data_min) / (data_max - data_min) for x in data]
        
        # Compare trajectories (position, velocity, acceleration)
        time = np.linspace(0, 1, 100)
        
        # Create figure to compare
        fig, axs = plt.subplots(3, 1, figsize=(12, 15))
        
        # Position comparison (first joint) - NORMALIZED
        default_pos_norm = normalize_data(default_data[0][0])
        optimized_pos_norm = normalize_data(optimized_data[0][0])
        
        axs[0].plot(time, default_pos_norm, 'r--', label='Default Position (Normalized)')
        axs[0].plot(time, optimized_pos_norm, 'b-', label='Optimized Position (Normalized)')
        axs[0].set_title('Normalized Position Comparison (Joint 1)')
        axs[0].set_ylabel('Position (Normalized)')
        axs[0].grid(True)
        axs[0].legend()
        
        # Velocity comparison - NORMALIZED
        default_vel_norm = normalize_data(default_data[0][1])
        optimized_vel_norm = normalize_data(optimized_data[0][1])
        
        axs[1].plot(time, default_vel_norm, 'r--', label='Default Velocity (Normalized)')
        axs[1].plot(time, optimized_vel_norm, 'b-', label='Optimized Velocity (Normalized)')
        axs[1].set_title('Normalized Velocity Comparison (Joint 1)')
        axs[1].set_ylabel('Velocity (Normalized)')
        axs[1].grid(True)
        axs[1].legend()
        
        # Acceleration comparison - NORMALIZED
        default_acc_norm = normalize_data(default_data[0][2])
        optimized_acc_norm = normalize_data(optimized_data[0][2])
        
        axs[2].plot(time, default_acc_norm, 'r--', label='Default Acceleration (Normalized)')
        axs[2].plot(time, optimized_acc_norm, 'b-', label='Optimized Acceleration (Normalized)')
        axs[2].set_title('Normalized Acceleration Comparison (Joint 1)')
        axs[2].set_xlabel('Time')
        axs[2].set_ylabel('Acceleration (Normalized)')
        axs[2].grid(True)
        axs[2].legend()
        
        # Add extra info about the scales
        fig.text(0.5, 0.01, 
                 f"Default velocity range: [{min(default_data[0][1]):.2f}, {max(default_data[0][1]):.2f}] | " +
                 f"Optimized velocity range: [{min(optimized_data[0][1]):.2f}, {max(optimized_data[0][1]):.2f}]",
                 ha='center', fontsize=10)
        
        plt.tight_layout(rect=[0, 0.03, 1, 1])  # Adjust layout to make room for the text
        plt.show()
        
        print("Comparison complete!")
        
    except Exception as e:
        progress_window.destroy()
        messagebox.showerror("Comparison Error", f"Error during comparison: {str(e)}")