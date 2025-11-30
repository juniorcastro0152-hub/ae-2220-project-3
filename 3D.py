from Cosmobee3D import Cosmobee

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import quaternion

try:
    # Use imageio-ffmpeg if available so users don't need a system ffmpeg
    # matplotlib's animation writer will use rcParams['animation.ffmpeg_path'] if set
    import imageio_ffmpeg as _imageio_ffmpeg
    ffmpeg_exe = _imageio_ffmpeg.get_ffmpeg_exe()
    import matplotlib
    matplotlib.rcParams['animation.ffmpeg_path'] = ffmpeg_exe
    _USING_IMAGEIO_FFMPEG = True
except Exception:
    _USING_IMAGEIO_FFMPEG = False
from matplotlib.patches import Wedge, FancyArrowPatch
from matplotlib.lines import Line2D
import time
import argparse

if __name__ == "__main__":
    # Read arguments (if any) from command line
    parser = argparse.ArgumentParser(description="Simulate Cosmobee trajectory")
    parser.add_argument('--save', type=str, help="Save the animation to MP4 with a given filename")
    parser.add_argument('--fps', type=int, default=30, help="Frames per second for the animation")
    parser.add_argument('--only-animation', action='store_true', help="Only shows the animation, no additional plots")
    args = parser.parse_args()

    # Generate a random starting orientation
    starting_euler = np.radians(np.array([np.random.uniform(low=-180.0, high=180.0), np.random.uniform(low=-90.0, high=90.0), np.random.uniform(low=-180.0, high=180.0)]))
    roll, pitch, yaw = starting_euler[0], starting_euler[1], starting_euler[2]
    # Generate a random target orientation
    target_euler = starting_euler + np.radians(np.array([np.random.uniform(low=-90.0, high=90.0), np.random.uniform(low=-45.0, high=45.0), np.random.uniform(low=-90.0, high=90.0)]))
    target_roll, target_pitch, target_yaw = target_euler[0], target_euler[1], target_euler[2]
    # Generate a starting angular velocity
    starting_angular_velocity = np.radians(np.random.uniform(low=-100.0, high=100.0, size=(3,))) # Starting with some random angular velocity
    roll_rate, pitch_rate, yaw_rate = starting_angular_velocity[0], starting_angular_velocity[1], starting_angular_velocity[2]

    # Create a Cosmobee instance
    bee = Cosmobee(yaw, pitch, roll, 0.0, 0.0, 0.0)  # Starting at random orientation, random angular velocity
    bee.set_target_orientation(target_yaw, target_pitch, target_roll)  # Target is the random target orientation
    
    # Simulation
    t = 0.0
    dt = 1.0 / args.fps
    orientation_history = []
    x_rw_omega_history = []
    y_rw_omega_history = []
    z_rw_omega_history = []
    x_rw_alpha_history = []
    y_rw_alpha_history = []
    z_rw_alpha_history = []
    roll_history = []
    pitch_history = []
    yaw_history = []
    omega_history = []
    alpha_history = []
    angular_momentum_history = []
    t_history = []

    while not bee.reached_orientation() and t < 10.0:
        orientation_history.append(bee.orientation)
        x_rw_omega_history.append(bee.x_reaction_wheel.angular_velocity)
        y_rw_omega_history.append(bee.y_reaction_wheel.angular_velocity)
        z_rw_omega_history.append(bee.z_reaction_wheel.angular_velocity)
        x_rw_alpha_history.append(bee.x_reaction_wheel.torque / bee.x_reaction_wheel.inertia)
        y_rw_alpha_history.append(bee.y_reaction_wheel.torque / bee.y_reaction_wheel.inertia)
        z_rw_alpha_history.append(bee.z_reaction_wheel.torque / bee.z_reaction_wheel.inertia)
        roll, pitch, yaw = quaternion.as_euler_angles(bee.orientation)
        roll_history.append(roll)
        pitch_history.append(pitch)
        yaw_history.append(yaw)
        omega_history.append(bee.angular_velocity.copy())
        alpha_history.append(bee.angular_acceleration.copy())
        angular_momentum_history.append(bee.get_angular_momentum())
        t_history.append(t)
        bee.update(dt=dt)
        t += dt

    print(f"Simulation completed in {t:.2f} seconds of simulated time over {len(t_history)} steps.")

    # Set up the figure and axis for animation
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_axis_off()
    ax.set_box_aspect([1,1,1])
    ax.set_title('Cosmobee 3D Orientation, t=0.00s')

    def update(frame):
        ax.cla()
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_axis_off()
        # Update the title with the current frame time in seconds
        if frame < len(t_history):
            ax.set_title('Cosmobee 3D Orientation, t={:.2f}s'.format(t_history[frame]))
        else:
            ax.set_title('Cosmobee 3D Orientation')

        orientation = orientation_history[frame]
        # Define body axes
        body_x = np.array([1, 0, 0])
        body_y = np.array([0, 1, 0])
        body_z = np.array([0, 0, 1])

        # Show body axes in global frame
        R = quaternion.as_rotation_matrix(orientation)
        global_x = R @ body_x
        global_y = R @ body_y
        global_z = R @ body_z
        origin = np.array([0, 0, 0])
        ax.quiver(*origin, *global_x, color='r', length=0.8, normalize=True, label='Body X')
        ax.quiver(*origin, *global_y, color='g', length=0.8, normalize=True, label='Body Y')
        ax.quiver(*origin, *global_z, color='b', length=0.8, normalize=True, label='Body Z')

        # Draw a cube representing the Cosmobee body aligned with the body axes (solid axes)
        plot_oriented_cube(ax, R, center=origin, side=0.5, color='k', alpha=0.25, linewidth=1.5, label='Body Cube')

        # Show target orientation axes (target_orientation is already in the global frame)
        target_orientation = bee.target_orientation
        R_target = quaternion.as_rotation_matrix(target_orientation)
        global_x_target = R_target @ body_x
        global_y_target = R_target @ body_y
        global_z_target = R_target @ body_z
        ax.quiver(*origin, *global_x_target, color='r', linestyle='dashed', length=0.8, normalize=True, label='Target X')
        ax.quiver(*origin, *global_y_target, color='g', linestyle='dashed', length=0.8, normalize=True, label='Target Y')
        ax.quiver(*origin, *global_z_target, color='b', linestyle='dashed', length=0.8, normalize=True, label='Target Z')

        return ax,


def plot_oriented_cube(ax, R, center=np.array([0.0, 0.0, 0.0]), side=1.0, color='k', alpha=0.15, linewidth=1.5, linestyle='-', label=None):
    """Plot a wireframe cube centered at `center` with rotation `R` applied.

    - `R` is a 3x3 rotation matrix (numpy array)
    - `side` is the full length of a cube side
    - The cube edges are drawn using `ax.plot`
    """
    # Half side length
    s = side / 2.0
    # Define vertices in the body frame (center at origin)
    vertices = np.array([
        [-s, -s, -s],  # 0
        [ s, -s, -s],  # 1
        [ s,  s, -s],  # 2
        [-s,  s, -s],  # 3
        [-s, -s,  s],  # 4
        [ s, -s,  s],  # 5
        [ s,  s,  s],  # 6
        [-s,  s,  s],  # 7
    ])

    # Apply rotation and translation to get global vertices
    global_vertices = (R @ vertices.T).T + center

    # Edge pairs (by index into vertices)
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7),  # vertical edges
    ]

    # Draw edges
    draw_label = label is not None
    for i, j in edges:
        p1 = global_vertices[i]
        p2 = global_vertices[j]
        xs = [p1[0], p2[0]]
        ys = [p1[1], p2[1]]
        zs = [p1[2], p2[2]]
        if draw_label:
            ax.plot(xs, ys, zs, color=color, alpha=alpha, linewidth=linewidth, linestyle=linestyle, label=label)
            draw_label = False
        else:
            ax.plot(xs, ys, zs, color=color, alpha=alpha, linewidth=linewidth, linestyle=linestyle)


ani = animation.FuncAnimation(fig, update, frames=len(t_history),
                                blit=False, interval=1000/args.fps, repeat_delay=2000)

# Make a static legend on the figure (not the axis) so it isn't cleared during frame updates
legend_handles = [
    Line2D([0], [0], color='r', lw=2, label='Body X'),
    Line2D([0], [0], color='g', lw=2, label='Body Y'),
    Line2D([0], [0], color='b', lw=2, label='Body Z'),
    Line2D([0], [0], color='r', lw=2, linestyle='--', label='Target X'),
    Line2D([0], [0], color='g', lw=2, linestyle='--', label='Target Y'),
    Line2D([0], [0], color='b', lw=2, linestyle='--', label='Target Z'),
]
fig.legend(handles=legend_handles, loc='upper right')
fig.tight_layout()

# If --only-animation argument is provided, show only the animation and exit
if args.only_animation:
    plt.show()
    exit(0)

# Add plots for reaction wheel angular velocities
fig_rw, ax_rw = plt.subplots()
ax_rw.plot(t_history, x_rw_omega_history, 'r-', label='X Reaction Wheel Angular Velocity', alpha=0.6)
ax_rw.plot(t_history, y_rw_omega_history, 'g-', label='Y Reaction Wheel Angular Velocity', alpha=0.6)
ax_rw.plot(t_history, z_rw_omega_history, 'b-', label='Z Reaction Wheel Angular Velocity', alpha=0.6)
ax_rw.set_xlabel('Time (s)')
ax_rw.set_ylabel('Reaction Wheel Angular Velocity (rad/s)')
ax_rw.set_title('Reaction Wheel Angular Velocity (rad/s) vs. Time (s)')
ax_rw.legend()
ax_rw.grid()

# Add plots for reaction wheel angular accelerations
fig_rwa, ax_rwa = plt.subplots()
ax_rwa.plot(t_history, x_rw_alpha_history, 'r-', label='X Reaction Wheel Angular Acceleration', alpha=0.6)
ax_rwa.plot(t_history, y_rw_alpha_history, 'g-', label='Y Reaction Wheel Angular Acceleration', alpha=0.6)
ax_rwa.plot(t_history, z_rw_alpha_history, 'b-', label='Z Reaction Wheel Angular Acceleration', alpha=0.6)
ax_rwa.set_xlabel('Time (s)')
ax_rwa.set_ylabel('Reaction Wheel Angular Acceleration (rad/s²)')
ax_rwa.set_title('Reaction Wheel Angular Acceleration (rad/s²) vs. Time (s)')
ax_rwa.legend()
ax_rwa.grid()

# Add plots for angular velocity of entire Cosmobee
fig_omega, ax_omega = plt.subplots()
omega_history = np.array(omega_history)
ax_omega.plot(t_history, omega_history[:, 0], 'r-', label='X Rate (rad/s)', alpha=0.6)
ax_omega.plot(t_history, omega_history[:, 1], 'g-', label='Y Rate (rad/s)', alpha=0.6)
ax_omega.plot(t_history, omega_history[:, 2], 'b-', label='Z Rate (rad/s)', alpha=0.6)
ax_omega.set_xlabel('Time (s)')
ax_omega.set_ylabel('Angular Velocity (rad/s)')
ax_omega.set_title('Cosmobee Angular Velocity (rad/s) vs. Time (s)')
ax_omega.legend()
ax_omega.grid()

# Add plotsfor angular acceleration of entire Cosmobee
fig_alpha, ax_alpha = plt.subplots()
alpha_history = np.array(alpha_history)
ax_alpha.plot(t_history, alpha_history[:, 0], 'r-', label='X Acceleration (rad/s²)', alpha=0.6)
ax_alpha.plot(t_history, alpha_history[:, 1], 'g-', label='Y Acceleration (rad/s²)', alpha=0.6)
ax_alpha.plot(t_history, alpha_history[:, 2], 'b-', label='Z Acceleration (rad/s²)', alpha=0.6)
ax_alpha.set_xlabel('Time (s)')
ax_alpha.set_ylabel('Angular Acceleration (rad/s²)')
ax_alpha.set_title('Cosmobee Angular Acceleration (rad/s²) vs. Time (s)')
ax_alpha.legend()
ax_alpha.grid()

# Add plots for angular momentum of entire Cosmobee
fig_H, ax_H = plt.subplots()
angular_momentum_history = np.array(angular_momentum_history)
ax_H.plot(t_history, angular_momentum_history[:, 0], 'r-', label='H_x (kg·m²/s)', alpha=0.6)
ax_H.plot(t_history, angular_momentum_history[:, 1], 'g-', label='H_y (kg·m²/s)', alpha=0.6)
ax_H.plot(t_history, angular_momentum_history[:, 2], 'b-', label='H_z (kg·m²/s)', alpha=0.6)
ax_H.set_xlabel('Time (s)')
ax_H.set_ylabel('Angular Momentum (kg·m²/s)')
ax_H.set_title('Cosmobee Angular Momentum (kg·m²/s) vs. Time (s)')
ax_H.legend()
ax_H.grid()

# Add plots for Euler angles over time
fig_euler, ax_euler = plt.subplots()
ax_euler.plot(t_history, np.degrees(roll_history), 'r-', label='Roll (degrees)', alpha=0.6)
ax_euler.plot(t_history, np.degrees(pitch_history), 'g-', label='Pitch (degrees)', alpha=0.6)
ax_euler.plot(t_history, np.degrees(yaw_history), 'b-', label='Yaw (degrees)', alpha=0.6)
ax_euler.set_xlabel('Time (s)')
ax_euler.set_ylabel('Euler Angles (degrees)')
ax_euler.set_title('Cosmobee Euler Angles (degrees) vs. Time (s)')
ax_euler.legend()
ax_euler.grid()

# Save the animation to MP4 if --save argument is provided
if args.save:
    output_filename = args.save
    print(f"Saving animation to {output_filename}...")
    start_time = time.time()
    # Prefer a concrete writer instance that will use the bundled ffmpeg if available
    try:
        if _USING_IMAGEIO_FFMPEG:
            writer = animation.FFMpegWriter(fps=args.fps)
        else:
            # Fall back to the default ffmpeg writer which requires system ffmpeg
            writer = 'ffmpeg'
    except Exception:
        writer = 'ffmpeg'
    ani.save(output_filename, writer=writer)
    end_time = time.time()
    print(f"Animation saved to {output_filename} in {end_time - start_time:.2f} seconds.")
plt.show()