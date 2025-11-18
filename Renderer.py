import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from Trajectory import Trajectory

class Renderer:
    def __init__(self, x_history, y_history, theta_history, thrust_history, t_history):
        self.x_history = x_history
        self.y_history = y_history
        self.theta_history = theta_history
        self.thrust_history = thrust_history
        self.t_history = t_history

    def plot_trajectory(self, trajectory: Trajectory):
        # Plots the provided trajectory points on a 2D plot.
        # Backwards-compatible: if called with no axes, plot on the current axes.
        try:
            ax = plt.gca()
        except Exception:
            ax = None

        xs = trajectory.points[:, 0]
        ys = trajectory.points[:, 1]
        if ax is not None:
            ax.plot(xs, ys, 'r--', label='Trajectory')
        else:
            # Fallback: create a new figure (rare)
            plt.figure()
            plt.plot(xs, ys, 'r--', label='Trajectory')

    def render(self, trajectory: Trajectory = None):
        """Create an animation of the history.

        If `trajectory` is provided, the trajectory will be drawn on the same
        axes used for the animation so they appear together.
        """
        with plt.style.context('seaborn-dark'):
            fig, ax = plt.subplots()
            # If a trajectory was provided, plot it on these axes
            if trajectory is not None:
                # draw trajectory on the animation axes
                ax.plot(trajectory.points[:, 0], trajectory.points[:, 1], 'r--', label='Trajectory')
            ax.set_xlim(min(self.x_history) - 1, max(self.x_history) + 1)
            ax.set_ylim(min(self.y_history) - 1, max(self.y_history) + 1)
            line, = ax.plot([], [], 'b-', lw=2)
            point, = ax.plot([], [], 'ro', markersize=8)

            def init():
                line.set_data([], [])
                point.set_data([], [])
                return line, point

            def update(frame):
                line.set_data(self.x_history[:frame], self.y_history[:frame])
                point.set_data(self.x_history[frame-1], self.y_history[frame-1])
                return line, point

            ani = animation.FuncAnimation(fig, update, frames=len(self.t_history),
                                          init_func=init, blit=True, interval=100)

            plt.xlabel('X Position')
            plt.ylabel('Y Position')
            plt.title('Cosmobee Trajectory')
            plt.grid()
            plt.show()