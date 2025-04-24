import tkinter as tk
from particles import TrajectoryOptimizer
import matplotlib.pyplot as plt

from mp_code.helper_fcns.utils import EndEffector, rotm_to_euler



# === Main ===
def main():
    EE_pose = [0.5, 0.5, 0.5]
    optimizer = TrajectoryOptimizer()
    optimizer.initialize_particle_cloud()
    # optimizer.optimization()
    optimizer.visualizer()

if __name__ == "__main__":
    main()