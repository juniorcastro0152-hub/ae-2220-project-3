from Cosmobee2D import Cosmobee

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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

    # Create a Cosmobee instance
    bee = Cosmobee(x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0, omega=0.0)
    
    # Simulation
    t = 0.0
    dt = 1.0 / args.fps
    x_history = []
    y_history = []
    theta_history = []
    vel_x_history = []
    vel_y_history = []
    omega_history = []
    target_history = []
    control_history = []
    thruster_right_history = []
    thruster_left_history = []
    thruster_up_history = []
    thruster_down_history = []
    wheel_omega_history = []
    wheel_angle_history = []
    t_history = []

    # Make a trajectory
    # Path 1: A rectangular path
    # start_point = (0, 0)
    # corner_bl = (1.5, 0)
    # corner_tl = (1.5, 3)
    # corner_tr = (8.5, 3)
    # corner_br = (8.5, 0)
    # end_point = (10, 0)
    # num_points_segment = 100
    # x1 = np.linspace(start_point[0], corner_bl[0], num_points_segment)
    # y1 = np.full_like(x1, start_point[1])
    # x2 = np.full(num_points_segment, corner_bl[0])
    # y2 = np.linspace(corner_bl[1], corner_tl[1], num_points_segment)
    # x3 = np.linspace(corner_tl[0], corner_tr[0], num_points_segment)
    # y3 = np.full_like(x3, corner_tl[1])
    # x4 = np.full(num_points_segment, corner_tr[0])
    # y4 = np.linspace(corner_tr[1], corner_br[1], num_points_segment)
    # x5 = np.linspace(corner_br[0], end_point[0], num_points_segment)
    # y5 = np.full_like(x5, corner_br[1])
    # x_traj = np.concatenate((x1, x2, x3, x4, x5))
    # y_traj = np.concatenate((y1, y2, y3, y4, y5))

    # Path 2: A wavy line
    x_traj = np.linspace(0, 10, 1000)
    y_traj = 1 * x_traj * np.sin(x_traj)
    z_traj = np.zeros_like(x_traj)
    # Make theta_traj to be the angle of the tangent to the trajectory
    dx = np.gradient(x_traj)
    dy = np.gradient(y_traj)
    # Compute tangent angle for the path and wrap to [0, 2*pi)
    theta_traj = np.arctan2(dy, dx) % (2 * np.pi)
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
        thruster_right_history.append(bee.thruster_right.thrust)
        thruster_left_history.append(bee.thruster_left.thrust)
        thruster_up_history.append(bee.thruster_up.thrust)
        thruster_down_history.append(bee.thruster_down.thrust)
        t_history.append(t)
        bee.update(dt=dt)
        wheel_omega_history.append(bee.reaction_wheel.angular_velocity)
        wheel_angle_history.append(bee.reaction_wheel.angle)
        t += dt

    # Make a matplotlib animation of the trajectory and the history
    fig, ax = plt.subplots(1, 2, figsize=(12, 6))
    ax[0].plot(x_traj, y_traj, 'r--', label='Desired Trajectory', alpha=0.2)
    line, = ax[0].plot([], [], 'b-', label="Actual Trajectory", lw=2)
    target_point, = ax[0].plot([], [], 'go', label="Current Target", markersize=8)
    point, = ax[0].plot([], [], 'ro', label="Current Position", markersize=8)
    title_ax = fig.add_axes([0, 0.92, 1.0, 0.08], frameon=False)
    title_ax.axis('off')
    text = title_ax.text(0.5, 0.65, '', ha='center', va='center', zorder=1000)
    ax[0].set_xlabel('X Position (m)')
    ax[0].set_ylabel('Y Position (m)')
    ax[0].grid()
    ax[0].axis('equal') 
    ax[0].legend(loc='upper left')

    # Plot a box showing the Cosmobee
    bee_box, = ax[1].plot([], [], 'k-')
    # Local (close-up) desired trajectory relative to the Cosmobee body frame
    local_traj_line, = ax[1].plot([], [], 'r--', label="Desired Trajectory", alpha=0.2)
    # Local (close-up) actual trajectory (vehicle history) shown in the same frame
    local_actual_line, = ax[1].plot([], [], 'b-', label="Actual Trajectory", lw=2)
    target_vector = ax[1].plot([], [], 'g-', label="Target Heading", lw=2)[0]
    bee_vector = ax[1].plot([], [], 'r-', label="Current Heading", lw=2)[0]
    local_target_point, = ax[1].plot([], [], 'go', label="Current Target", markersize=6)
    # Create thruster arrows using FancyArrowPatch
    thruster_arrows = []
    arrow_kwargs = dict(mutation_scale=10, linewidth=2, color='m', zorder=6)
    for _ in range(4):
        arr = FancyArrowPatch((0, 0), (0, 0), **arrow_kwargs)
        arr.set_animated(True)
        ax[1].add_patch(arr)
        thruster_arrows.append(arr)

    text2 = title_ax.text(0.75, 0.65, '', ha='center', va='center', zorder=1000)
    # Align text positions with subplot centers
    pos0 = ax[0].get_position()
    pos1 = ax[1].get_position()
    center0 = pos0.x0 + pos0.width / 2.0
    center1 = pos1.x0 + pos1.width / 2.0
    text.set_x(center0)
    text2.set_x(center1)

    # Set fixed view limits and enforce equal aspect ratio
    closeup_size = 2.0
    ax[1].set_xlim(-closeup_size, closeup_size)
    ax[1].set_ylim(-closeup_size, closeup_size)
    ax[1].set_aspect('equal', adjustable='box')
    ax[1].set_autoscale_on(False)   # Wedges autoscale to huge sizes otherwise

    # Reaction wheel visual: a circle composed of four 90deg wedges (two purple, two black)
    # Diameter is half of the Cosmobee side length
    wheel_radius = bee.side_length * 0.25
    wheel_wedges: list[Wedge] = []
    wheel_colors = ['purple', 'black', 'purple', 'black']
    for i in range(4):
        th1 = i * 90
        th2 = th1 + 90
        w = Wedge((0.0, 0.0), wheel_radius, th1, th2,
                  facecolor=wheel_colors[i], edgecolor='k', lw=0.6, zorder=5,
                  transform=ax[1].transData)
        ax[1].add_patch(w)
        w.set_animated(True)
        wheel_wedges.append(w)
    ax[1].set_xlabel('Local X (m)')
    ax[1].set_ylabel('Local Y (m)')
    ax[1].grid()
    ax[1].legend(loc='upper left')

    # Mark remaining animated artists
    for a in [line, target_point, point, local_traj_line, local_actual_line, local_target_point, bee_box, text, text2, target_vector, bee_vector, *thruster_arrows, *wheel_wedges]:
        a.set_animated(True)

    frame_skip = 1 # Skip frames for faster animation if needed, set to 1 to show all frames
    def update(frame):
        frame *= frame_skip
        line.set_data(x_history[:frame], y_history[:frame])
        target_point.set_data([target_history[frame][0]], [target_history[frame][1]])
        point.set_data([x_history[frame]], [y_history[frame]])
        text.set_text('Cosmobee Trajectory, t={:.2f}s'.format(t_history[frame]))

        # Draw the box representing the Cosmobee with the rotation based on theta
        angle = theta_history[frame]
        box_size = bee.side_length
        box_x = np.array([-box_size/2, box_size/2, box_size/2, -box_size/2, -box_size/2])
        box_y = np.array([-box_size/2, -box_size/2, box_size/2, box_size/2, -box_size/2])
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                 [np.sin(angle),  np.cos(angle)]])
        box_rotated = rot_matrix @ np.vstack((box_x, box_y))
        bee_box.set_data(box_rotated[0, :],
                         box_rotated[1, :])
        target_vector.set_data([0, box_size * np.cos(target_history[frame][3])],
                               [0, box_size * np.sin(target_history[frame][3])])
        bee_vector.set_data([0, box_size * np.cos(angle)],
                    [0, box_size * np.sin(angle)])

        # Draw desired trajectory in local frame
        curr_x = x_history[frame]
        curr_y = y_history[frame]
        local_x = x_traj - curr_x
        local_y = y_traj - curr_y
        local_traj_line.set_data(local_x, local_y)
        # Draw recent actual trajectory in local frame
        if frame > 0:
            # Fill history to current frame
            hx = np.array(x_history[:frame])
            hy = np.array(y_history[:frame])
            local_actual_line.set_data(hx - curr_x, hy - curr_y)
        else:
            local_actual_line.set_data([], [])
        # Show target point in local frame
        tx, ty = target_history[frame][0], target_history[frame][1]
        local_target_point.set_data([tx - curr_x], [ty - curr_y])

        # Draw thrusters
        thrusters = [bee.thruster_right, bee.thruster_left, bee.thruster_up, bee.thruster_down]
        bases = [] # Will be a list of [x,y] base positions
        U = [] # X components of thruster vectors
        V = [] # Y components of thruster vectors
        # Set recorded thruster magnitudes for this frame
        thr_vals = [
            thruster_right_history[frame] if frame < len(thruster_right_history) else 0.0,
            thruster_left_history[frame] if frame < len(thruster_left_history) else 0.0,
            thruster_up_history[frame] if frame < len(thruster_up_history) else 0.0,
            thruster_down_history[frame] if frame < len(thruster_down_history) else 0.0,
        ]
        for thr_obj, thr_val in zip(thrusters, thr_vals):
            # Set thruster base position in world coordinates using thruster mount position
            base_local = np.array([thr_obj.position[0], thr_obj.position[1]])
            base_world = rot_matrix @ base_local
            # Calculate thruster direction in world coordinates using thruster angle
            dir_local = np.array([np.cos(thr_obj.angle), np.sin(thr_obj.angle)])
            dir_world = rot_matrix @ dir_local
            # Thruster magnitude is normalized to max thrust and scaled by side length for the visualization
            length = (thr_val / thr_obj.max_thrust) * bee.side_length
            u, v = dir_world * length
            # Set respective thruster base and vector components
            bases.append(base_world)
            U.append(u)
            V.append(v)
        bases_arr = np.array(bases)
        # Update arrow patches positions: start at base, end at base+vector
        for arr, base_world, u, v in zip(thruster_arrows, bases_arr, U, V):
            start = (base_world[0], base_world[1])
            end = (base_world[0] + u, base_world[1] + v)
            arr.set_positions(start, end)
        
        # Draw reaction wheel
        scale = 1.0 # scale factor to make rotation clear (if it spins too slow or fast for the animation)
        rot_deg = 0.0
        if frame < len(wheel_angle_history):
            rot_deg = np.degrees(wheel_angle_history[frame] * scale)
        for i, w in enumerate(wheel_wedges):
            # Wedges are each 90 degrees apart, so offset by i*90
            w.set_theta1(i * 90 + rot_deg)
            w.set_theta2(i * 90 + rot_deg + 90)
        text2.set_text('Cosmobee Control, t={:.2f}s'.format(t_history[frame]))
        return line, target_point, point, local_traj_line, local_actual_line, local_target_point, bee_box, target_vector, bee_vector, *thruster_arrows, *wheel_wedges, text, text2
    ani = animation.FuncAnimation(fig, update, frames=len(t_history) // frame_skip,
                                    blit=True, interval=1, repeat_delay=2000)

    # If --only-animation argument is provided, show only the animation and exit
    if args.only_animation:
        plt.show()
        exit(0)

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

    plt.figure()
    plt.subplot(2, 1, 1)
    distances = []
    for xi, yi in zip(x_history, y_history):
        dists = np.linalg.norm(trajectory[:, :2] - np.array([xi, yi]), axis=1)
        min_dist = np.min(dists)
        distances.append(min_dist)
    plt.plot(t_history, distances, label='Position Error', alpha=0.6)
    plt.xlabel('Time (s)')
    plt.ylabel('Distance to Trajectory (m)')
    plt.legend(loc=1)
    plt.title('Position Error Over Time')
    plt.grid()
    plt.subplot(2, 1, 2)
    wheel_omega_history = np.array(wheel_omega_history)
    wheel_angular_momentum = bee.reaction_wheel.inertia * wheel_omega_history
    cosmobee_angular_momentum = bee.Izz * np.array(omega_history)
    total_angular_momentum = np.zeros_like(cosmobee_angular_momentum)
    plt.plot(t_history, wheel_angular_momentum, label='Reaction Wheel Angular Momentum', alpha=0.6)
    plt.plot(t_history, cosmobee_angular_momentum, label='Cosmobee Angular Momentum', alpha=0.6)
    plt.plot(t_history, total_angular_momentum, label='Total Angular Momentum', alpha=0.6)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Momentum (kg·m²/s)')
    plt.legend(loc=1)
    plt.title('Angular Momentum Over Time')
    plt.grid()

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