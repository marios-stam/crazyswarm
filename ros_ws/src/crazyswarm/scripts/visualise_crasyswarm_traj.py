from fileinput import filename
import os
import numpy as np
from uav_trajectory import Trajectory
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def visualise_traj(file):
    tr = Trajectory()

    tr.loadcsv(file)
    print("duration:", tr.duration)
    t_space = np.linspace(0, tr.duration, 100)

    xs = np.zeros(len(t_space)-1)
    ys = np.zeros(len(t_space)-1)
    zs = np.zeros(len(t_space)-1)

    for i, t in enumerate(t_space[:-1]):
        evaluation = tr.eval(t)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]

        print("t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)

        xs[i], ys[i], zs[i] = x, y, z

    # Visualise the trajectory in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, 'r')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


print("Current directory:", os.getcwd())

file = "/home/marios/crazyswarm/ros_ws/src/crazyswarm/scripts/figure8.csv"

file = "/home/marios/piecewise_pole.csv"
visualise_traj(file)

file = "/home/marios/Pol_matrix.csv"
visualise_traj(file)
