from trajectory import *


def main():
    """Main function that runs the simulation"""

    traj = MultiAxisTrajectoryGenerator(method="s-curve",
                                        interval=[0,20],
                                        ndof=1,
                                        start_pos=[0],
                                        final_pos=[15])
    
    # generate trajectory
    t = traj.generate(nsteps=200)

    # plot trajectory
    traj.plot()


if __name__ == "__main__":
    main()