from Cosmobee import Cosmobee
from Renderer import Renderer

import numpy as np

if __name__ == "__main__":
    # Create a Cosmobee instance
    bee = Cosmobee(x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0, omega=0.0)
    
    # Set thruster forces
    thrust_vector = [1.0, 0.5, 0]  # Example thrust vector in global frame
    bee.set_thrust(thrust_vector)
    
    # Simulation
    x_history = []
    y_history = []
    theta_history = []
    thrust_history = []
    t_history = []

    # Make a trajectory
    x_traj = np.linspace(0, 10, 100)
    y_traj = np.linspace(0, 5, 100)
    z_traj = np.zeros(100)
    theta_traj = np.zeros(100)
    trajectory = np.vstack((x_traj, y_traj, z_traj, theta_traj)).T

    # Temporary set thrust for testing
    thrust_vector = np.array([2, 1, 0])
    bee.set_thrust(thrust_vector)

    while False:
        # TODO: Add the pure pursuit controller here to follow the trajectory
        bee.update(dt=0.1)
        x_history.append(bee.x)
        y_history.append(bee.y)
        theta_history.append(bee.theta)
        thrust_history.append(thrust_vector)
        t_history.append(len(t_history) * 0.1)

    # Rendering
    renderer = Renderer(x_history, y_history, theta_history, thrust_history, t_history)
    renderer.render()
    