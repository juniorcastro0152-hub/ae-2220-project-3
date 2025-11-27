"""Cosmobee spacecraft model (Python port of the MATLAB class).

This module provides a minimal, idiomatic Python implementation of the
Cosmobee class that was originally written in MATLAB. It includes state
variables, thruster configuration, and a method to set thruster commands.
"""

import numpy as np
import enum

from PID import PID

class Thruster:
    def __init__(self, max_thrust: float, position: np.array, angle: float):
        self.max_thrust = max_thrust
        self.thrust = 0.0
        self.position = position
        self.angle = angle

    def set_thrust(self, thrust: float):
        self.thrust = np.clip(thrust, 0.0, self.max_thrust)

class ReactionWheel:
    def __init__(self, max_torque: float, inertia: float, axis: np.array):
        self.min_torque = -max_torque
        self.max_torque = max_torque
        self.inertia = inertia
        self.axis = axis
        self.angular_velocity = 0.0
        # Integrated wheel angle (radians). This is the wheel's rotation
        # angle in the wheel frame and is updated in `update()` so callers
        # can read a continuously integrated angle without re-integrating
        # histories themselves.
        self.angle = 0.0
        self.torque = 0.0

    def set_torque(self, torque: float):
        self.torque = np.clip(torque, self.min_torque, self.max_torque)

    def update(self, dt: float):
        # angular acceleration = torque / inertia
        # negative because of equal-and-opposite reaction
        self.angular_velocity -= (self.torque / self.inertia) * dt
        # Integrate the wheel angle using the (new) angular velocity so the
        # ReactionWheel maintains its own angle state.
        self.angle += self.angular_velocity * dt
        self.angle = self.angle

class Cosmobee:
    """Simple 2D rigid-body model with 4 thrusters and one reaction wheel.
    """

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        omega: float = 0.0,
        dim: int = 2,
    ) -> None:
        # Rigid body properties
        self.mass: float = 20 # kg
        self.side_length: float = 1 # meters
        self.Izz: float = (1.0 / 12.0) * self.mass * self.side_length ** 2

        # State variables
        self.x: float = float(x)
        self.y: float = float(y)
        self.theta: float = float(theta)
        self.vx: float = float(vx)
        self.vy: float = float(vy)
        self.omega: float = float(omega)
        self.dim: int = dim

        # Set max vehicle performance limits for safety
        self.max_velocity: float = 1 # m/s
        self.max_angular_velocity: float = np.pi # rad/s

        # Controllers
        self.x_pid = PID(Kp=100.0, Ki=0.0, Kd=0.0)
        self.y_pid = PID(Kp=100.0, Ki=0.0, Kd=0.0)
        self.theta_pid = PID(Kp=40.0, Ki=0.0, Kd=0.0)

        # Set the initial setpoints to current velocities
        self.x_pid.set_setpoint(self.vx)
        self.y_pid.set_setpoint(self.vy)
        self.theta_pid.set_setpoint(self.omega)

        self.x_control: float = 0.0
        self.y_control: float = 0.0
        self.theta_control: float = 0.0

        # Trajectory and pure pursuit algorithm
        self.trajectory = None
        self.lookahead_distance: float = 0.25 # meters
        self.current_target_index: int = 0

        # Individual thruster properties
        self.max_thrust: float = 20.0 # N

        # Thruster positions (relative to COM) and fixed angles (radians)
        # Each thruster can produce thrust up to max_thrust in its fixed direction
        self.thruster_right = Thruster(self.max_thrust, np.array((0.5, 0.0, 0.0)), 0.0)
        self.thruster_left = Thruster(self.max_thrust, np.array((-0.5, 0.0, 0.0)), np.pi)
        self.thruster_up = Thruster(self.max_thrust, np.array((0.0, 0.5, 0.0)), np.pi / 2.0)
        self.thruster_down = Thruster(self.max_thrust, np.array((0.0, -0.5, 0.0)), -np.pi / 2.0)

        # Reaction wheel properties
        self.max_torque: float = 50.0   # N*m
        inertia_rw = 5 * 0.2 ** 2  # kg*m^2, assuming hoop of 5kg mass and 0.2m radius
        self.reaction_wheel = ReactionWheel(max_torque=self.max_torque, inertia=inertia_rw, axis=np.array((0, 0, 1)))

    def __repr__(self) -> str:  # pragma: no cover - small convenience method
        return (
            f"Cosmobee(x={self.x}, y={self.y}, theta={self.theta}, vx={self.vx}, "
            f"vy={self.vy}, omega={self.omega})"
        )

    def set_trajectory(self, trajectory: np.array) -> None:
        """Set the trajectory for the Cosmobee to follow."""
        if trajectory.shape[1] != 4:
            raise ValueError("Trajectory must be of shape (N, 4) for 2D Cosmobee.")
        self.trajectory = trajectory
        self.current_target_index = 0
    
    def get_local_to_global_rotation_matrix(self) -> np.array:
        """Get the rotation matrix from local to global frame based on current theta."""
        c = np.cos(self.theta)
        s = np.sin(self.theta)
        return np.array([[c, -s],
                         [s,  c]])
    
    def set_thrust(self, global_thrust_vector: np.array) -> None:
        """Set thruster forces based on desired global thrust vector."""
        if global_thrust_vector.shape != (3,):
            raise ValueError("Thrust vector must be a 3D vector.")

        if self.dim == 2:
            global_thrust_vector = global_thrust_vector[:2]

        # Turn global thrust into local frame
        R = self.get_local_to_global_rotation_matrix()
        local_thrust = np.linalg.inv(R).dot(global_thrust_vector)
        
        # Set the targets for each thruster so that the PID can handle it
        self.thruster_right.set_thrust(max(0.0, -local_thrust[0]))
        self.thruster_left.set_thrust(max(0.0, local_thrust[0]))
        self.thruster_up.set_thrust(max(0.0, -local_thrust[1]))
        self.thruster_down.set_thrust(max(0.0, local_thrust[1]))

    def set_torque(self, torque: float) -> None:
        """Set reaction wheel torque."""
        self.reaction_wheel.set_torque(torque)

    def update_new_target_position(self) -> np.array:
        """Get the next target position from the trajectory based on lookahead distance and the current position."""
        if self.trajectory is None:
            raise ValueError("No trajectory set for Cosmobee.")
        # Find the next point that is at least lookahead_distance away using Euclidean distance
        norms = np.linalg.norm(self.trajectory[:, :3] - np.array([self.x, self.y, 0]), axis=1)
        # Filter to only points ahead of the current target index
        ahead_indices = np.where(norms >= self.lookahead_distance)[0]
        ahead_indices = ahead_indices[ahead_indices >= self.current_target_index]
        if len(ahead_indices) > 0:
            self.current_target_index = ahead_indices[0]
            return self.trajectory[self.current_target_index]
        # If no such point exists, return the last point since we've reached the end
        self.current_target_index = len(self.trajectory) - 1
        return self.trajectory[-1]

    def update(self, dt: float) -> None:
        # Get the next target position
        target_position = self.update_new_target_position()

        # Calculate the target velocities based on the current position and target position
        # Average velocity to reach the target in dt seconds
        vel_gain = 0.6
        angular_gain = 1.3
        target_vx = max(min(vel_gain*(target_position[0] - self.x) / self.lookahead_distance, self.max_velocity), -self.max_velocity)
        target_vy = max(min(vel_gain*(target_position[1] - self.y) / self.lookahead_distance, self.max_velocity), -self.max_velocity)

        delta_theta = target_position[3] - self.theta
        if delta_theta > np.pi:
            delta_theta -= 2 * np.pi
        elif delta_theta < -np.pi:
            delta_theta += 2 * np.pi

        target_omega = max(min(angular_gain*delta_theta, self.max_angular_velocity), -self.max_angular_velocity)

        # Update PID controllers
        self.x_pid.set_setpoint(target_vx)
        self.y_pid.set_setpoint(target_vy)
        self.theta_pid.set_setpoint(target_omega)

        # Transform the local velocities to global frame for PID update
        R = self.get_local_to_global_rotation_matrix()
        global_velocity = R.dot(np.array([self.vx, self.vy]))
        self.x_control = self.x_pid.update(global_velocity[0], dt) # thrust in x
        self.y_control = self.y_pid.update(global_velocity[1], dt) # thrust in y
        self.theta_control = self.theta_pid.update(self.omega, dt) # torque in z

        # See whether to override controls based on max velocity limits
        # If velocity and x_control have the same sign, we are speeding up
        if (global_velocity[0] * self.x_control > 0) and (abs(global_velocity[0]) >= self.max_velocity):
            self.x_control = 0.0
        if (global_velocity[1] * self.y_control > 0) and (abs(global_velocity[1]) >= self.max_velocity):
            self.y_control = 0.0
        if (self.omega * self.theta_control > 0) and (abs(self.omega) >= self.max_angular_velocity):
            self.theta_control = 0.0

        # Set thruster forces and reaction wheel torque
        self.set_thrust(np.array([self.x_control, self.y_control, 0.0]))
        self.set_torque(self.theta_control)

        # Update state based on dynamics
        # Thrust is reversed because thrusters push against the spacecraft
        total_thrust_x = -(self.thruster_right.thrust * np.cos(self.thruster_right.angle) +
                          self.thruster_left.thrust * np.cos(self.thruster_left.angle))
        total_thrust_y = -(self.thruster_up.thrust * np.sin(self.thruster_up.angle) +
                          self.thruster_down.thrust * np.sin(self.thruster_down.angle))

        self.vx += total_thrust_x / self.mass * dt
        self.vy += total_thrust_y / self.mass * dt

        # Update reaction wheel (integrate wheel state) and apply reaction 
        # torque to the spacecraft body.
        self.reaction_wheel.update(dt)
        self.omega += self.reaction_wheel.torque / self.Izz * dt

        # These are always in global frame
        self.theta += self.omega * dt
        # Normalize theta to be within 0 to 2pi
        self.theta = self.theta % (2 * np.pi)

        # Update the global position
        R = self.get_local_to_global_rotation_matrix()
        global_velocity = R.dot(np.array([self.vx, self.vy]))
        self.x += global_velocity[0] * dt
        self.y += global_velocity[1] * dt

    def reached_goal(self) -> bool:
        """Return True if the Cosmobee has reached the end of its trajectory."""
        if self.trajectory is None:
            return False
        theta_diff = abs(self.theta - self.trajectory[-1, 3])
        if theta_diff > np.pi:
            theta_diff = abs(theta_diff - 2 * np.pi)
        return self.current_target_index >= len(self.trajectory) - 1 and np.linalg.norm(
            np.array([self.x, self.y]) - self.trajectory[-1, :2]) % (2 * np.pi) < 0.01 and theta_diff < 0.01 and \
            abs(self.vx) < 0.01 and abs(self.vy) < 0.01 and abs(self.omega) < 0.01