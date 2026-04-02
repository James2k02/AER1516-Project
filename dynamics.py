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
    
    def move_cost(self, state1: State, state2: State) -> float:
        """Cost (time) to move between two states."""
        # TODO: Implement (placeholder: just Euclidean distance / speed)
        pass
    
    def can_hop_over(self, obstacle, state1: State, state2: State) -> bool:
        """Can robot hop over this obstacle when moving from state1 to state2?"""
        pass
    
    def trajectory(self, state1: State, state2: State, num_substeps: int = 10) -> List[State]:
        """Return interpolated trajectory for collision checking."""
        # TODO: Implement (placeholder: linear interpolation)

        # Defining the differential drive kinematics of the robot

        dt = 1.0 / num_substeps # time step

        trajectory = np.zeros((num_substeps, 3))

        x, y, theta = state1.x, state1.y, state1.theta

        for i in range(num_substeps):
            # Distance and direction to target state2
            dx = state2.x - x
            dy = state2.y - y
            dtheta = math.atan2(dy, dx) - theta

            # Wrap angular difference to (-pi, pi]
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

            # simple controller
            v = self.V
            omega = 2.0 * dtheta   # proportional control

            # integrate
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt
            theta = (theta + np.pi) % (2 * np.pi) - np.pi

            # Store
            trajectory[i, 0] = x
            trajectory[i, 1] = y
            trajectory[i, 2] = theta

            # Early stop if close to target
            if np.sqrt(dx*dx + dy*dy) < 0.2:
                trajectory = trajectory[:i+1]
                break

        # Convert to List[State] (keep compatibility with your system)
        return [State(p[0], p[1], p[2]) for p in trajectory]
    
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

class rurtle_dynamics(RobotDynamics):
    """ To be implemented"""

class LIMP_dynamics(RobotDynamics):
    """ maybe? two legged dynamics"""