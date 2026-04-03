from typing import List, Tuple, NamedTuple
import math
import numpy as np
 
 
class State(NamedTuple):
    """
    Represents a configuration in the configuration space.
    
    Attributes:
        x (float): X position in map (0 to map_width)
        y (float): Y position in map (0 to map_height)
        theta (float): Orientation in radians [0, 2π)
    """
    x: float
    y: float
    theta: float

    # UTILITY METHODS
    
    def position(self) -> Tuple[float, float]:
        """Return (x, y) position tuple."""
        return (self.x, self.y)
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """Return as regular tuple (x, y, theta)."""
        return (self.x, self.y, self.theta)
    
    def distance_to(self, other: 'State') -> float:
        """
        Calculate Euclidean distance to another state (x, y only).
        
        Args:
            other: Target state
        
        Returns:
            Euclidean distance between (x, y) positions
        """
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
 
    def state_distance(s1: State, s2: State) -> float:
        """
        Compute Euclidean distance between two states (x, y only).
        """
        return s1.distance_to(s2)


class RobotDynamics:
    """
    Interface for plugging in custom robot dynamics.
    Dynamics team fills in the implementations.
    """
    def __init__(self, max_vel = 1.0, max_angular_vel = 0.5, robot_radius = 0.5):
        self.max_vel = max_vel
        self.max_angular_vel = max_angular_vel
        self.robot_radius = robot_radius
    
    def move_cost(self, state1: State, state2: State) -> float:
        """Cost (time) to move between two states."""
        # TODO: Implement (placeholder: just Euclidean distance / speed)
        pass
    
    def can_hop_over(self, obstacle, state1: State, state2: State) -> bool:
        """Can robot hop over this obstacle when moving from state1 to state2?"""
        pass

    # def hop_over(Self, state1: State, state2: State) ->

    def robot_controller(self, nearest_state: State, target_state: State):
        """
        Simple proportional controller to generate (v, omega)
        that drives the robot toward the target.
        """

        dx = target_state.x - nearest_state.x
        dy = target_state.y - nearest_state.y

        # Distance and heading
        lin_dist = math.sqrt(dx*dx + dy*dy)
        desired_theta = math.atan2(dy, dx)

        # Heading error
        dtheta = desired_theta - nearest_state.theta
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

        # Gains (tune these!)
        v_gain = 1.0
        omega_gain = 2.0

        # Control
        v = v_gain * lin_dist
        omega = omega_gain * dtheta

        # Clip
        v = np.clip(v, -self.max_vel, self.max_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)

        return v, omega
    
    def trajectory(self, state: State, v: float, omega: float, num_substeps: int = 10):
        """
        Forward simulate robot dynamics given constant (v, omega).
        Returns Nx3 numpy array.
        """

        dt = 1.0 / num_substeps

        trajectory = np.zeros((num_substeps, 3))

        x, y, theta = state.x, state.y, state.theta

        for i in range(num_substeps):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt

            # Wrap angle
            theta = (theta + np.pi) % (2 * np.pi) - np.pi

            trajectory[i] = [x, y, theta]

        return trajectory
    
    def obstacle_position_at_time(self, obstacle_id: int, t: float) -> Tuple:
        """Return obstacle position (x, y, w, h) at time t."""
        # TODO: Implement (placeholder: static obstacle)
        pass
    
    def get_max_speed(self) -> float:
        """Return maximum robot speed."""
        # TODO: Implement
        pass
    
    def get_hop_speed(self) -> float:
        """Return speed when hopping over obstacles."""
        # TODO: Implement
        pass

class jumping_turtle_dynamics(RobotDynamics):
    """ To be implemented"""

class turtle_dynamics(RobotDynamics):
    """ To be implemented"""

class LIMP_dynamics(RobotDynamics):
    """ maybe? two legged dynamics"""