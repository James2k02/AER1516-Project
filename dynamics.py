from typing import List, Tuple, NamedTuple
import math
import numpy as np
from config import GOAL_SUCCESS_THRESH 
 
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
    
    @staticmethod
    def state_distance(s1: 'State', s2: 'State') -> float:
        """
        Compute Euclidean distance between two states (x, y only).
        """
        return s1.distance_to(s2)


class RobotDynamics:
    """
    Interface for plugging in custom robot dynamics.
    Dynamics team fills in the implementations.
    """
    def __init__(self):
        self.max_vel = 3.0 # m/s
        self.max_angular_vel = 1.5
        self.robot_radius = 0.15 # m

        self.grid = None
        self.static_obstacles = []
        self.dynamic_obstacles = []
    
    def move_cost(self, state1: State, state2: State) -> float:
        """Cost (time) to move between two states."""
        pass

    def can_hop_over(self, obstacle, state1: State, state2: State) -> bool:
        """Can robot hop over this obstacle when moving from state1 to state2?"""
        pass

    def robot_controller(self, nearest_state: State, target_state: State):
        """
        Simple proportional controller to generate (v, omega)
        that drives the robot toward the target.
        """

        dx = target_state.x - nearest_state.x
        dy = target_state.y - nearest_state.y

        lin_dist = math.sqrt(dx*dx + dy*dy)
        desired_theta = math.atan2(dy, dx)

        dtheta = desired_theta - nearest_state.theta
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

        v_gain = 1.0
        omega_gain = 2.5

        v = v_gain * lin_dist
        omega = omega_gain * dtheta

        v = np.clip(v, -self.max_vel, self.max_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)

        return v, omega
    
    def trajectory(self, state: State, v: float, omega: float, num_substeps: int = 10):
        """
        Forward simulate robot dynamics given constant (v, omega).
        Returns Nx3 numpy array.
        """

        dt = 0.5 / num_substeps

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
    
    def is_valid_position(self, r, c, size):
        for i in range(size):
            for j in range(size):
                rr = r + i
                cc = c + j

                if rr < 0 or rr >= self.grid.shape[0] or cc < 0 or cc >= self.grid.shape[1]:
                    return False

                if self.grid[rr, cc] == 1:
                    return False

        return True
    
    def simulate_trajectory(self, path, goal_tolerance = GOAL_SUCCESS_THRESH):
        """
        Simulate robot following a path using controller + dynamics.

        Args:
            path: list of State waypoints
            goal_tolerance: distance threshold to consider waypoint reached

        Returns:
            np.ndarray of shape (N, 3): [x, y, theta]
        """

        if path is None or len(path) == 0:
            return None

        full_traj = []

        current_state = path[0]

        for target_state in path[1:]:
            while current_state.distance_to(target_state) > goal_tolerance:
                v, omega = self.robot_controller(current_state, target_state)
                traj_segment = self.trajectory(current_state, v, omega)

                if traj_segment is None or len(traj_segment) == 0:
                    break

                for x, y, theta in traj_segment:
                    full_traj.append([x, y, theta])

                x, y, theta = traj_segment[-1]
                current_state = State(x, y, theta)

        return np.array(full_traj)    
    
    def get_hop_speed(self) -> float:
        """Return speed when hopping over obstacles."""
        pass

class jumping_turtle_dynamics(RobotDynamics):
    """ To be implemented"""

class turtle_dynamics(RobotDynamics):
    """ To be implemented"""

class LIMP_dynamics(RobotDynamics):
    """ maybe? two legged dynamics"""