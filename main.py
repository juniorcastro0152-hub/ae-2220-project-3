from Cosmobee import Cosmobee

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

if __name__ == "__main__":
    # Create a Cosmobee instance
    bee = Cosmobee(x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0, omega=0.0)
    
    # Simulation
    t = 0.0
    dt = 0.1
    x_history = []
    y_history = []
    theta_history = []
    vel_x_history = []
    vel_y_history = []
    omega_history = []
    target_history = []
    control_history = []
    t_history = []

    # Make a trajectory
    x_traj = np.linspace(0, 100, 1000)
    y_traj = x_traj * np.sin(0.1 * x_traj)
    z_traj = np.zeros(1000)
    # Make theta_traj to be the angle of the tangent to the trajectory
    dx = np.gradient(x_traj)
    dy = np.gradient(y_traj)
    theta_traj = np.arctan2(dy, dx) + 2 * np.pi # add 2pi to avoid negative angles
    # Turn the theta_traj to be between 0 and 2pi
    theta_traj = theta_traj % (2 * np.pi)
    trajectory = np.vstack((x_traj, y_traj, z_traj, theta_traj)).T

    # Set the trajectory
    bee.set_trajectory(trajectory)

    while not bee.reached_goal() and t < 1000.0:
        x_history.append(bee.x)
        y_history.append(bee.y)
        theta_history.append(bee.theta)
        vel_x_history.append(bee.vx)
        vel_y_history.append(bee.vy)
        omega_history.append(bee.omega)
        target_history.append(trajectory[bee.current_target_index])
        control_history.append(np.array([bee.x_control, bee.y_control, bee.theta_control]))
        t_history.append(t)
        bee.update(dt=dt)
        t += dt

    # Make a matplotlib animation of the trajectory and the history
    fig, ax = plt.subplots(1, 2, figsize=(12, 6))
    ax[0].plot(x_traj, y_traj, 'r--', label='Desired Trajectory', alpha=0.2)
    line, = ax[0].plot([], [], 'b-', lw=2)
    target_point, = ax[0].plot([], [], 'go', markersize=8)
    point, = ax[0].plot([], [], 'ro', markersize=8)
    text = ax[0].text(0.5, 1.01, '', horizontalalignment='center', verticalalignment='bottom', transform=ax[0].transAxes)
    ax[0].set_xlabel('X Position (m)')
    ax[0].set_ylabel('Y Position (m)')
    ax[0].grid()

    # Plot a box showing the Cosmobee
    bee_viz, = ax[1].plot([], [], 'k-', markersize=8)
    bee_vector = ax[1].plot([], [], 'r-', lw=2)[0]
    target_vector = ax[1].plot([], [], 'g-', lw=2)[0]
    text2 = ax[1].text(0.5, 1.01, '', horizontalalignment='center', verticalalignment='bottom', transform=ax[1].transAxes)
    ax[1].set_xlabel('X Position (m)')
    ax[1].set_ylabel('Y Position (m)')
    ax[1].grid()
    ax[1].set_xlim(-2, 2)
    ax[1].set_ylim(-2, 2)

    frame_skip = 3 # Skip frames for faster animation
    def init():
        line.set_data([], [])
        point.set_data([], [])
        target_point.set_data([], [])
        return line, point, target_point
    def update(frame):
        frame *= frame_skip
        line.set_data(x_history[:frame], y_history[:frame])
        target_point.set_data([target_history[frame-1][0]], [target_history[frame-1][1]])
        point.set_data([x_history[frame-1]], [y_history[frame-1]])
        text.set_text('Cosmobee Trajectory, t={:.2f}s'.format(t_history[frame-1]))

        # Draw the box representing the Cosmobee with the rotation based on theta
        angle = theta_history[frame-1]
        box_size = bee.side_length
        box_x = np.array([-box_size/2, box_size/2, box_size/2, -box_size/2, -box_size/2])
        box_y = np.array([-box_size/2, -box_size/2, box_size/2, box_size/2, -box_size/2])
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                 [np.sin(angle),  np.cos(angle)]])
        box_rotated = rot_matrix @ np.vstack((box_x, box_y))
        bee_viz.set_data(box_rotated[0, :],
                         box_rotated[1, :])
        bee_vector.set_data([0, box_size * np.cos(angle)],
                            [0, box_size * np.sin(angle)])
        target_vector.set_data([0, box_size * np.cos(target_history[frame-1][3])],
                               [0, box_size * np.sin(target_history[frame-1][3])])
        text2.set_text('Cosmobee Orientation, t={:.2f}s'.format(t_history[frame-1]))
        return line, point, target_point
    ani = animation.FuncAnimation(fig, update, frames=len(t_history) // frame_skip,
                                    init_func=init, blit=False, interval=1, repeat_delay=2000)

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t_history, vel_x_history, label='Vx', alpha=0.6)
    plt.plot(t_history, vel_y_history, label='Vy', alpha=0.6)
    plt.hlines(bee.max_velocity, 0, t_history[-1], color='k', linestyle='--', label='Max Velocity')
    plt.hlines(-bee.max_velocity, 0, t_history[-1], color='k', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend(loc=1)
    plt.title('Velocities Over Time (Cosmobee Body Frame)')
    plt.grid()
    plt.subplot(2, 1, 2)
    plt.plot(t_history, omega_history, label='Omega', alpha=0.6)
    plt.hlines(bee.max_angular_velocity, 0, t_history[-1], color='k', linestyle='--', label='Max Angular Velocity')
    plt.hlines(-bee.max_angular_velocity, 0, t_history[-1], color='k', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.legend(loc=1)
    plt.title('Angular Velocity Over Time')
    plt.grid()

    plt.figure()
    plt.subplot(2, 1, 1)
    control_history = np.array(control_history)
    plt.plot(t_history, control_history[:, 0], label='X Control', alpha=0.6)
    plt.plot(t_history, control_history[:, 1], label='Y Control', alpha=0.6)
    plt.hlines(bee.max_thrust, 0, t_history[-1], color='k', linestyle='--', label='Max Thrust')
    plt.hlines(-bee.max_thrust, 0, t_history[-1], color='k', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust Control (N)')
    plt.legend(loc=1)
    plt.title('Control Outputs Over Time (Translational)')
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(t_history, control_history[:, 2], label='Theta Control', alpha=0.6)
    plt.hlines(bee.max_torque, 0, t_history[-1], color='k', linestyle='--', label='Max Torque')
    plt.hlines(-bee.max_torque, 0, t_history[-1], color='k', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque Control (Nm)')
    plt.legend(loc=1)
    plt.title('Angular Control Over Time (Rotational)')
    plt.grid()
    plt.show()