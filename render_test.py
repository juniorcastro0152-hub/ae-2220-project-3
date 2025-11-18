from Trajectory import Trajectory
from Renderer import Renderer
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Simple test of the Trajectory class
    points = [
        [0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0, np.pi/4],
        [2.0, 0.0, 0.0, np.pi/2],
    ]
    traj = Trajectory(points, threshold=0.5)
    renderer = Renderer([], [], [], [], [])

    renderer.plot_trajectory(traj)
    plt.title("Trajectory Test")
    plt.show()