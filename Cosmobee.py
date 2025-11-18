"""Cosmobee spacecraft model (Python port of the MATLAB class).

This module provides a minimal, idiomatic Python implementation of the
Cosmobee class that was originally written in MATLAB. It includes state
variables, thruster configuration, and a method to set thruster commands.
"""

import numpy as np
import enum

from PID import PID
from Trajectory import Trajectory

# Write an enum for thruster indices for better readability
class Thruster(enum.IntEnum):
    THRUSTER_1 = 1
    THRUSTER_2 = 2
    THRUSTER_3 = 3
    THRUSTER_4 = 4

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
        self.torque = 0.0

    def set_torque(self, torque: float):
        self.torque = np.clip(torque, self.min_torque, self.max_torque)

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
    ) -> None:
        # Rigid body properties
        self.mass: float = 20 # kg
        self.side_length: float = 1.0 # meters
        self.Izz: float = (1.0 / 12.0) * self.mass * self.side_length ** 2

        # State variables
        self.x: float = float(x)
        self.y: float = float(y)
        self.theta: float = float(theta)
        self.vx: float = float(vx)
        self.vy: float = float(vy)
        self.omega: float = float(omega)

        # Controllers
        self.x_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)
        self.y_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)
        self.theta_pid = PID(Kp=1.0, Ki=0.0, Kd=0.0)

        # Trajectory
        self.trajectory = None

        # Individual thruster properties
        self.max_thrust: float = 10.0 # N

        # Thruster positions (relative to COM) and fixed angles (radians)
        # Each thruster can produce thrust up to max_thrust in its fixed direction
        self.thruster_right = Thruster(self.max_thrust, np.array((0.5, 0.0, 0.0)), 0.0)
        self.thruster_left = Thruster(self.max_thrust, np.array((-0.5, 0.0, 0.0)), np.pi)
        self.thruster_up = Thruster(self.max_thrust, np.array((0.0, 0.5, 0.0)), np.pi / 2.0)
        self.thruster_down = Thruster(self.max_thrust, np.array((0.0, -0.5, 0.0)), -np.pi / 2.0)

        # Reaction wheel properties
        self.max_torque: float = 1.0   # N*m
        self.reaction_wheel = ReactionWheel(min_torque=self.min_torque, max_torque=self.max_torque, inertia=0.01, axis=np.array((0, 0, 1)))

    def __repr__(self) -> str:  # pragma: no cover - small convenience method
        return (
            f"Cosmobee(x={self.x}, y={self.y}, theta={self.theta}, vx={self.vx}, "
            f"vy={self.vy}, omega={self.omega})"
        )
    
    def set_trajectory(self, trajectory: Trajectory) -> None:
        """Set the trajectory for the Cosmobee to follow."""
        self.trajectory = trajectory
    
    def get_local_to_global_rotation_matrix(self) -> np.array:
        """Get the rotation matrix from local to global frame based on current theta."""
        c = np.cos(self.theta)
        s = np.sin(self.theta)
        return np.array([[c, -s],
                         [s,  c]])
    
    def set_thrust(self, global_thrust_vector: np.array) -> None:
        """Set thruster forces based on desired global thrust vector."""
        if len(global_thrust_vector) != 2:
            raise ValueError("Thrust vector must be a 2D vector.")

        # Turn global thrust into local frame
        R = self.get_local_to_global_rotation_matrix()
        local_thrust = np.linalg.inv(R).dot(global_thrust_vector)
        
        # Set the targets for each thruster so that the PID can handle it
        self.thruster_right.set_thrust(max(0.0, local_thrust[0]))
        self.thruster_left.set_thrust(max(0.0, -local_thrust[0]))
        self.thruster_up.set_thrust(max(0.0, local_thrust[1]))
        self.thruster_down.set_thrust(max(0.0, -local_thrust[1]))

    def set_torque(self, torque: float) -> None:
        """Set reaction wheel torque."""
        self.reaction_wheel.set_torque(torque)
