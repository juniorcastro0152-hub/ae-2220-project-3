"""
Cosmobee 3D dynamics model.
This module defines a simple 3D rigid-body model of the Cosmobee spacecraft
with 3 reaction wheels for attitude control.
"""

import numpy as np
import quaternion

from PID import PID
from ReactionWheel import ReactionWheel

class Cosmobee:
    """3D rigid-body model with 3 reaction wheels."""

    def __init__(
        self,
        yaw: float = 0.0,
        pitch: float = 0.0,
        roll: float = 0.0,
        yaw_rate: float = 0.0,
        pitch_rate: float = 0.0,
        roll_rate: float = 0.0,
    ) -> None:
        # Rigid body properties
        self.mass: float = 20 # kg
        self.side_length: float = 1 # meters
        I_diag = (1.0 / 6.0) * self.mass * self.side_length ** 2
        self.I = np.diag([I_diag, I_diag, I_diag])  # Inertia tensor for a uniform cube

        # State variables
        self.orientation = quaternion.from_euler_angles([roll, pitch, yaw])
        self.target_orientation = self.orientation
        
        # ZYX transformation matrix from Euler rates to angular velocity
        T = np.matrix([
            [1, 0, -np.sin(pitch)],
            [0, np.cos(roll), np.cos(pitch)*np.sin(roll)],
            [0, -np.sin(roll), np.cos(pitch)*np.cos(roll)]
        ])

        self.angular_velocity = T @ np.array([[roll_rate], [pitch_rate], [yaw_rate]])
        self.angular_acceleration = np.zeros((3,1))

        # Controllers
        self.z_pid = PID(Kp=100.0, Ki=0.0, Kd=-20.0)
        self.y_pid = PID(Kp=100.0, Ki=0.0, Kd=-20.0)
        self.x_pid = PID(Kp=100.0, Ki=0.0, Kd=-20.0)

        # Set the setpoints to 0
        self.z_pid.set_setpoint(0.0)
        self.y_pid.set_setpoint(0.0)
        self.x_pid.set_setpoint(0.0)

        self.z_control: float = 0.0
        self.y_control: float = 0.0
        self.x_control: float = 0.0

        # Reaction wheel properties
        self.max_torque: float = 20.0   # N*m
        inertia_rw = 5 * 0.2 ** 2  # kg*m^2, assuming hoop of 5kg mass and 0.2m radius
        self.z_reaction_wheel = ReactionWheel(max_torque=self.max_torque, inertia=inertia_rw, axis=np.array([0, 0, 1]))
        self.y_reaction_wheel = ReactionWheel(max_torque=self.max_torque, inertia=inertia_rw, axis=np.array([0, 1, 0]))
        self.x_reaction_wheel = ReactionWheel(max_torque=self.max_torque, inertia=inertia_rw, axis=np.array([1, 0, 0]))

    def __repr__(self) -> str:  # pragma: no cover - small convenience method
        return (
            f"Cosmobee(orientation={self.orientation}, "
            f"angular_velocity={self.angular_velocity.flatten()}, "
            f"x_rw_ang_vel={self.x_reaction_wheel.angular_velocity}, "
            f"y_rw_ang_vel={self.y_reaction_wheel.angular_velocity}, "
            f"z_rw_ang_vel={self.z_reaction_wheel.angular_velocity})"
        )
    
    def get_angular_momentum(self) -> np.array:
        """Compute total angular momentum of the Cosmobee system."""
        # Angular momentums of reaction wheels
        h_w = np.array([
            self.x_reaction_wheel.inertia * self.x_reaction_wheel.angular_velocity,
            self.y_reaction_wheel.inertia * self.y_reaction_wheel.angular_velocity,
            self.z_reaction_wheel.inertia * self.z_reaction_wheel.angular_velocity
        ]).reshape((3,1))

        # Total angular momentum
        H = self.I @ self.angular_velocity + h_w
        return H
    
    def set_target_orientation(self, yaw: float, pitch: float, roll: float) -> None:
        """Set target orientation in Euler angles (radians)."""
        self.target_orientation = quaternion.from_euler_angles([roll, pitch, yaw])

    def set_torque(self, torque: np.array) -> None:
        """Set reaction wheel torque."""
        if torque.shape != (3,):
            raise ValueError("Torque must be a 3D vector.")
        self.x_reaction_wheel.set_torque(torque[0])
        self.y_reaction_wheel.set_torque(torque[1])
        self.z_reaction_wheel.set_torque(torque[2])

    def reached_orientation(self, attitude_tolerance: float = 0.001, rate_tolerance: float = 0.001) -> bool:
        """Check if the Cosmobee has reached the target orientation within tolerances."""
        orientation_error = self.orientation.conjugate() * self.target_orientation

        # Unwinding check to ensure shortest rotation
        if orientation_error.w < 0:
            orientation_error = -orientation_error

        # Use vector part of pure quaternion as attitude error
        attitude_error = np.array([
            orientation_error.x,
            orientation_error.y,
            orientation_error.z
        ])
        angular_velocity_magnitude = np.linalg.norm(self.angular_velocity)

        return (np.linalg.norm(attitude_error) < attitude_tolerance) and (angular_velocity_magnitude < rate_tolerance)

    def update(self, dt: float) -> None:
        """Update the Cosmobee state over a time step dt."""
        # Compute orientation error
        orientation_error = self.orientation.conjugate() * self.target_orientation

        # Unwinding check to ensure shortest rotation
        if orientation_error.w < 0:
            orientation_error = -orientation_error

        # Use vector part of pure quaternion as attitude error
        attitude_error = np.array([
            orientation_error.x,
            orientation_error.y,
            orientation_error.z
        ])

        rate_error = np.asarray(self.angular_velocity).ravel()

        # PID control for each axis
        self.z_control = self.z_pid.update(dt, -attitude_error[2], rate_error[2])
        self.y_control = self.y_pid.update(dt, -attitude_error[1], rate_error[1])
        self.x_control = self.x_pid.update(dt, -attitude_error[0], rate_error[0])
        control_torque = np.array([self.x_control, self.y_control, self.z_control])

        # Set reaction wheel torques
        self.set_torque(control_torque)

        # Set control torque based on actual reaction wheel torques (respects max torque limits)
        control_torque = np.array([self.x_reaction_wheel.torque, self.y_reaction_wheel.torque, self.z_reaction_wheel.torque])

        # print(f"Orientation error: {orientation_error}, \tRate error: {rate_error}, \tControl torque: {control_torque}")

        # Update reaction wheels
        self.x_reaction_wheel.update(dt)
        self.y_reaction_wheel.update(dt)
        self.z_reaction_wheel.update(dt)

        # Dynamics update

        H = self.get_angular_momentum()

        # Gyroscopic torque (precession effect) due to changing wheel angular momentums
        gyroscopic_torque = np.cross(self.angular_velocity.reshape((3,)), H.reshape((3,)))

        # Compute angular acceleration
        angular_acceleration = np.linalg.inv(self.I) @ ( control_torque - gyroscopic_torque ).reshape((3,1))

        self.angular_acceleration = angular_acceleration

        # Update angular velocity
        self.angular_velocity += angular_acceleration * dt

        # Update orientation quaternion
        omega_matrix = np.matrix([
            [0, -self.angular_velocity[0, 0], -self.angular_velocity[1, 0], -self.angular_velocity[2, 0]],
            [self.angular_velocity[0, 0], 0, self.angular_velocity[2, 0], -self.angular_velocity[1, 0]],
            [self.angular_velocity[1, 0], -self.angular_velocity[2, 0], 0, self.angular_velocity[0, 0]],
            [self.angular_velocity[2, 0], self.angular_velocity[1, 0], -self.angular_velocity[0, 0], 0]
        ])

        q_dot = 0.5 * omega_matrix @ quaternion.as_float_array(self.orientation).reshape((4,1))
        self.orientation += quaternion.from_float_array(np.asarray(q_dot.flatten()).flatten() * dt)
        normalized = quaternion.as_float_array(self.orientation) / np.linalg.norm(quaternion.as_float_array(self.orientation))
        self.orientation = quaternion.from_float_array(normalized)