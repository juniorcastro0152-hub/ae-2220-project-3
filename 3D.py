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
    # Generate a starting angular velocity
    starting_angular_velocity = np.radians(np.random.uniform(low=0.0, high=0.0, size=(3,)))
    roll_rate, pitch_rate, yaw_rate = starting_angular_velocity[0], starting_angular_velocity[1], starting_angular_velocity[2]

    # Create a Cosmobee instance
    bee = Cosmobee(yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate)
    bee.set_target_orientation(0.0, 0.0, 0.0)  # Target orientation is aligned with global frame
    
    # Simulation
    t = 0.0
    dt = 1.0 / args.fps
    orientation_history = []
    x_rw_history = []
    y_rw_history = []
    z_rw_history = []
    t_history = []

    while not bee.reached_orientation() and t < 10.0:
        orientation_history.append(bee.orientation)
        x_rw_history.append(bee.x_reaction_wheel.angular_velocity)
        y_rw_history.append(bee.y_reaction_wheel.angular_velocity)
        z_rw_history.append(bee.z_reaction_wheel.angular_velocity)
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
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Cosmobee 3D Orientation')

    def update(frame):
        ax.cla()
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
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

        # Show target orientation axes (target_orientation is already in the global frame)
        target_orientation = bee.target_orientation
        R_target = quaternion.as_rotation_matrix(target_orientation)
        global_x_target = R_target @ body_x
        global_y_target = R_target @ body_y
        global_z_target = R_target @ body_z
        ax.quiver(*origin, *global_x_target, color='r', linestyle='dashed', length=0.8, normalize=True, label='Target X')
        ax.quiver(*origin, *global_y_target, color='g', linestyle='dashed', length=0.8, normalize=True, label='Target Y')
        ax.quiver(*origin, *global_z_target, color='b', linestyle='dashed', length=0.8, normalize=True, label='Target Z')

        ax.legend()

        return ax,

    ani = animation.FuncAnimation(fig, update, frames=len(t_history),
                                    blit=False, interval=1000/args.fps, repeat_delay=2000)

    # If --only-animation argument is provided, show only the animation and exit
    if args.only_animation:
        plt.show()
        exit(0)

    # Add plots for reaction wheel speeds
    fig_rw, ax_rw = plt.subplots()
    ax_rw.plot(t_history, x_rw_history, label='X Reaction Wheel Angular Velocity', alpha=0.6)
    ax_rw.plot(t_history, y_rw_history, label='Y Reaction Wheel Angular Velocity', alpha=0.6)
    ax_rw.plot(t_history, z_rw_history, label='Z Reaction Wheel Angular Velocity', alpha=0.6)
    ax_rw.set_xlabel('Time (s)')
    ax_rw.set_ylabel('Reaction Wheel Angular Velocity (rad/s)')
    ax_rw.set_title('Reaction Wheel Angular Velocity (rad/s) vs. Time (s)')
    ax_rw.legend()
    ax_rw.grid()

    # plt.figure()
    # plt.subplot(2, 1, 1)
    # plt.plot(t_history, vel_x_history, label='Vx', alpha=0.6)
    # plt.plot(t_history, vel_y_history, label='Vy', alpha=0.6)
    # plt.hlines(bee.max_velocity, 0, t_history[-1], color='k', linestyle='--', label='Max Velocity')
    # plt.hlines(-bee.max_velocity, 0, t_history[-1], color='k', linestyle='--')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Velocity (m/s)')
    # plt.legend(loc=1)
    # plt.title('Velocities Over Time (Cosmobee Body Frame)')
    # plt.grid()
    # plt.subplot(2, 1, 2)
    # plt.plot(t_history, omega_history, label='Omega', alpha=0.6)
    # plt.hlines(bee.max_angular_velocity, 0, t_history[-1], color='k', linestyle='--', label='Max Angular Velocity')
    # plt.hlines(-bee.max_angular_velocity, 0, t_history[-1], color='k', linestyle='--')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Angular Velocity (rad/s)')
    # plt.legend(loc=1)
    # plt.title('Angular Velocity Over Time')
    # plt.grid()

    # plt.figure()
    # plt.subplot(2, 1, 1)
    # control_history = np.array(control_history)
    # plt.plot(t_history, control_history[:, 0], label='X Control', alpha=0.6)
    # plt.plot(t_history, control_history[:, 1], label='Y Control', alpha=0.6)
    # plt.hlines(bee.max_thrust, 0, t_history[-1], color='k', linestyle='--', label='Max Thrust')
    # plt.hlines(-bee.max_thrust, 0, t_history[-1], color='k', linestyle='--')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Thrust Control (N)')
    # plt.legend(loc=1)
    # plt.title('Control Outputs Over Time (Translational)')
    # plt.grid()

    # plt.subplot(2, 1, 2)
    # plt.plot(t_history, control_history[:, 2], label='Theta Control', alpha=0.6)
    # plt.hlines(bee.max_torque, 0, t_history[-1], color='k', linestyle='--', label='Max Torque')
    # plt.hlines(-bee.max_torque, 0, t_history[-1], color='k', linestyle='--')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Torque Control (Nm)')
    # plt.legend(loc=1)
    # plt.title('Angular Control Over Time (Rotational)')
    # plt.grid()

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