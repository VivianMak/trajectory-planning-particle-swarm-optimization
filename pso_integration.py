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
                                    values=["min_jerk", "min_time", "min_energy", "combined"])
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
            interval=[0, 10],
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
    Creates a timestamped subdirectory for each run under 'trajectory_plots' directory.
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
            interval=[0, 10],
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
            interval=[0, 10],
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
        
        # Create directory structure for saving plots
        import os
        import datetime
        
        # Create base directory if it doesn't exist
        base_plots_dir = "trajectory_plots"
        os.makedirs(base_plots_dir, exist_ok=True)
        
        # Create a unique timestamp for this run
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create a subdirectory for this specific run
        run_dir_name = f"{metric}_{timestamp}"
        run_dir = os.path.join(base_plots_dir, run_dir_name)
        os.makedirs(run_dir, exist_ok=True)
        
        # Save run metadata to a text file
        with open(os.path.join(run_dir, "run_info.txt"), 'w') as f:
            f.write(f"Optimization run: {timestamp}\n")
            f.write(f"Metric: {metric}\n")
            f.write(f"Number of particles: {n_particles}\n")
            f.write(f"Number of iterations: {iterations}\n")
            f.write(f"Optimized parameters: t1={params['t1']:.3f}, t2={params['t2']:.3f}, v_max={params['v_max']:.3f}\n")
            f.write(f"Waypoints: Start={waypoints[0]}, End={waypoints[1]}\n")
        
        # Normalization function
        def normalize_data(data):
            data_min = min(data)
            data_max = max(data)
            if data_max == data_min:
                return [0.5] * len(data)  # Handle constant data
            return [(x - data_min) / (data_max - data_min) for x in data]
        
        # Time array for plotting
        time = np.linspace(0, 1, 100)
        
        # Show only Joint 1 in GUI (as before)
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
        
        # Save the main displayed plot
        main_plot_file = os.path.join(run_dir, "joint1_main_display.png")
        fig.savefig(main_plot_file)
        plt.show()  # Still show Joint 1 plot interactively
        
        # Generate and save plots for ALL joints
        for joint_idx in range(len(start_joint)):
            # Create a figure for this joint
            joint_fig, joint_axs = plt.subplots(3, 1, figsize=(12, 15))
            joint_fig.suptitle(f'Joint {joint_idx+1} Trajectory Comparison', fontsize=16)
            
            # Position comparison
            default_pos_norm = normalize_data(default_data[joint_idx][0])
            optimized_pos_norm = normalize_data(optimized_data[joint_idx][0])
            
            joint_axs[0].plot(time, default_pos_norm, 'r--', label='Default Position (Normalized)')
            joint_axs[0].plot(time, optimized_pos_norm, 'b-', label='Optimized Position (Normalized)')
            joint_axs[0].set_title(f'Normalized Position Comparison (Joint {joint_idx+1})')
            joint_axs[0].set_ylabel('Position (Normalized)')
            joint_axs[0].grid(True)
            joint_axs[0].legend()
            
            # Velocity comparison
            default_vel_norm = normalize_data(default_data[joint_idx][1])
            optimized_vel_norm = normalize_data(optimized_data[joint_idx][1])
            
            joint_axs[1].plot(time, default_vel_norm, 'r--', label='Default Velocity (Normalized)')
            joint_axs[1].plot(time, optimized_vel_norm, 'b-', label='Optimized Velocity (Normalized)')
            joint_axs[1].set_title(f'Normalized Velocity Comparison (Joint {joint_idx+1})')
            joint_axs[1].set_ylabel('Velocity (Normalized)')
            joint_axs[1].grid(True)
            joint_axs[1].legend()
            
            # Acceleration comparison
            default_acc_norm = normalize_data(default_data[joint_idx][2])
            optimized_acc_norm = normalize_data(optimized_data[joint_idx][2])
            
            joint_axs[2].plot(time, default_acc_norm, 'r--', label='Default Acceleration (Normalized)')
            joint_axs[2].plot(time, optimized_acc_norm, 'b-', label='Optimized Acceleration (Normalized)')
            joint_axs[2].set_title(f'Normalized Acceleration Comparison (Joint {joint_idx+1})')
            joint_axs[2].set_xlabel('Time')
            joint_axs[2].set_ylabel('Acceleration (Normalized)')
            joint_axs[2].grid(True)
            joint_axs[2].legend()
            
            # Add velocity ranges
            joint_fig.text(0.5, 0.01, 
                     f"Default velocity range: [{min(default_data[joint_idx][1]):.2f}, {max(default_data[joint_idx][1]):.2f}] | " +
                     f"Optimized velocity range: [{min(optimized_data[joint_idx][1]):.2f}, {max(optimized_data[joint_idx][1]):.2f}]",
                     ha='center', fontsize=10)
            
            plt.tight_layout(rect=[0, 0.03, 1, 1])
            
            # Save the plot to file
            joint_plot_file = os.path.join(run_dir, f"joint_{joint_idx+1}_comparison.png")
            joint_fig.savefig(joint_plot_file)
            plt.close(joint_fig)  # Close this figure to avoid displaying it
        
        # Additionally save the optimization history
        history_fig, history_ax = plt.subplots(figsize=(10, 6))
        history_ax.plot(fitness_history)
        history_ax.set_title(f'Fitness History - {metric}')
        history_ax.set_xlabel('Iteration')
        history_ax.set_ylabel('Fitness Value')
        history_ax.grid(True)
        
        # Save the history plot
        history_file = os.path.join(run_dir, "optimization_history.png")
        history_fig.savefig(history_file)
        plt.close(history_fig)
        
        # Also create a combined plot with absolute data (non-normalized) for analysis
        abs_fig, abs_axs = plt.subplots(3, len(start_joint), figsize=(16, 12))
        abs_fig.suptitle(f'Absolute Trajectory Data Comparison (All Joints)', fontsize=16)
        
        for joint_idx in range(len(start_joint)):
            # Use actual values (not normalized)
            # Position
            abs_axs[0, joint_idx].plot(time, default_data[joint_idx][0], 'r--', label='Default')
            abs_axs[0, joint_idx].plot(time, optimized_data[joint_idx][0], 'b-', label='Optimized')
            abs_axs[0, joint_idx].set_title(f'Joint {joint_idx+1} Position')
            abs_axs[0, joint_idx].grid(True)
            abs_axs[0, joint_idx].legend()
            
            # Velocity
            abs_axs[1, joint_idx].plot(time, default_data[joint_idx][1], 'r--', label='Default')
            abs_axs[1, joint_idx].plot(time, optimized_data[joint_idx][1], 'b-', label='Optimized')
            abs_axs[1, joint_idx].set_title(f'Joint {joint_idx+1} Velocity')
            abs_axs[1, joint_idx].grid(True)
            
            # Acceleration
            abs_axs[2, joint_idx].plot(time, default_data[joint_idx][2], 'r--', label='Default')
            abs_axs[2, joint_idx].plot(time, optimized_data[joint_idx][2], 'b-', label='Optimized')
            abs_axs[2, joint_idx].set_title(f'Joint {joint_idx+1} Acceleration')
            abs_axs[2, joint_idx].grid(True)
        
        # Set common labels for rows
        abs_axs[0, 0].set_ylabel('Position')
        abs_axs[1, 0].set_ylabel('Velocity')
        abs_axs[2, 0].set_ylabel('Acceleration')
        
        # Set common x-axis label for bottom row
        for j in range(len(start_joint)):
            abs_axs[2, j].set_xlabel('Time')
        
        plt.tight_layout()
        
        # Save the combined absolute plot
        abs_plot_file = os.path.join(run_dir, "all_joints_absolute_comparison.png")
        abs_fig.savefig(abs_plot_file)
        plt.close(abs_fig)
        
        # Inform the user where the plots are saved
        messagebox.showinfo("Plots Saved", 
                           f"Comparison plots for all joints have been saved to:\n\n{run_dir}\n\nMetric: {metric}\nTimestamp: {timestamp}")
        
        print("Comparison complete!")
        
    except Exception as e:
        progress_window.destroy()
        messagebox.showerror("Comparison Error", f"Error during comparison: {str(e)}")