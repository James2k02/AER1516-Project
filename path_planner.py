"""
RRT IMPLEMENTATION OVERVIEW
================================================================================

Key components work together in the following flow:
    1. Sample random configuration in free space
    2. Find nearest node in existing tree
    3. Steer from nearest node toward random sample (using robot dynamics)
    4. Collision check the new edge (considering static + dynamic obstacles)
    5. If collision-free, add node to tree
    6. [RRT* FUTURE] Rewire tree to optimize costs
    7. Check if goal is reached
    8. Extract and return path when goal found or iteration limit hit

COMPONENT BREAKDOWN & TODO LIST
================================================================================

TODO 1: STATE REPRESENTATION
--------
Purpose: Define how we represent a configuration in the configuration space
Details: 
    - State format: (x, y, theta) or (x, y) depending on holonomic assumption
    - Needs to be compatible with distance metrics and collision checking
    - Should be easily serializable for tree storage
Examples of what to implement:
    - State class/namedtuple
    - __eq__, __hash__ for tree operations
    - Distance metric function (Euclidean)

TODO 2: RANDOM SAMPLING
--------
Purpose: Generate random, collision-free configurations to guide tree exploration
Details:
    - Sample uniformly in rectangular map bounds
    - Option: Goal biasing (sample goal with probability p_goal, else uniform)
    - Ensure samples stay within map boundaries
    - Samples should be collision-free at time of sampling (static obstacles)
Examples of what to implement:
    - sample_random_config(map_bounds, p_goal=0.05)
    - sample_uniform(bounds)
    - boundary checking

TODO 3: NEAREST NEIGHBOR SEARCH
--------
Purpose: Find the closest existing node in the tree to a newly sampled config
Details:
    - Distance metric: Euclidean distance in 2D
    - Simple version: Linear search through all nodes O(n)
    - Optimized version: KD-tree for O(log n) lookups (future enhancement)
    - Return the closest node object (with position, parent, cost info)
Examples of what to implement:
    - nearest_node(tree, config)
    - distance function (Euclidean, with wrapping for angle if needed)
    - Tree node data structure

TODO 4: STEERING / LOCAL EDGE INTERPOLATION
--------
Purpose: Attempt to move robot from nearest node toward random sample
Details:
    - Define step_size or max_extension_length (tuning parameter)
    - Compute direction unit vector from nearest toward sample
    - Move step_size in that direction
    - Result may not reach the sample (that's OK, tree grows gradually)
    - **KEY: This is where robot dynamics plug in**
      * Simple version: straight-line interpolation at constant speed
      * Future: hopping vs. direct movement decisions, varying speeds
    - Return new candidate configuration
Examples of what to implement:
    - steer(start_state, target_state, step_size, dynamics_model)
    - direction computation
    - distance/speed integration (if dynamics affect travel time)

TODO 5: COLLISION CHECKING
--------
Purpose: Verify that a path segment is collision-free (most critical for safety)
Details:
    - **Static obstacles:** Check if line segment intersects rectangular obstacles
    - **Dynamic obstacles:** Check if segment collides with obstacles at the time
      the robot would traverse it (requires dynamics module input)
    - **Robot geometry:** Handle robot as point or with footprint/radius
    - Segment collision: Line-rectangle intersection tests
    - **NOTE:** This is where dynamic obstacle integration happens
      * Dynamics module tells us obstacle positions over time
      * We compute time robot spends on edge, check collisions at those times
Examples of what to implement:
    - is_collision_free_trajectory(start_state, end_state, static_obstacles, 
                       dynamic_obstacles, dynamics_model, t_start=0)
    - line_segment_intersects_rectangle(p1, p2, rect)
    - point_in_rectangle(point, rect)
    - obstacle_position_at_time(obstacle, t) [delegated to dynamics module]
    - trajectory interpolation for dense collision checking

TODO 6: EDGE COST COMPUTATION
--------
Purpose: Calculate cost (distance/time) to traverse an edge (needed for RRT*)
Details:
    - Simple version: Euclidean distance between nodes
    - Better version: Incorporate robot speed and dynamics
    - Account for hopping penalty if applicable
      * Direct movement: distance / normal_speed
      * Hopping: distance / hop_speed (slower)
      * Cost should reflect actual traversal time
    - Used for path optimization in RRT* rewiring
Examples of what to implement:
    - edge_cost(state1, state2, dynamics_model)
    - cost accumulation along path
    - speed lookup based on movement type

TODO 7: REWIRING (RRT* FUTURE ENHANCEMENT)
--------
Purpose: Optimize the tree by rerouting nodes through cheaper paths
Details:
    - After adding new node, find all nodes within radius r
    - For each neighbor, check if path through new node is cheaper
    - If cheaper, rewire: update parent pointer and accumulated cost
    - Radius decreases as tree grows: r = k * (log(n) / n)^(1/d)
      * n = tree size, d = dimension (2 for 2D)
      * k is a tuning parameter
    - Only attempt rewiring for collision-free paths
Examples of what to implement:
    - rewire_neighbors(tree, new_node, radius)
    - find_neighbors_in_radius(tree, node, radius)
    - cost comparison and parent updates
    - NOTE: Start with basic RRT, add this later for RRT*

TODO 8: GOAL DETECTION & TERMINATION
--------
Purpose: Determine when planning is complete and path is found
Details:
    - Goal region: distance threshold from goal point (e.g., 0.5 units)
    - Check if any node in tree is within threshold of goal
    - Termination conditions:
      * Found path to goal (hard stop or continue for optimization?)
      * Max iterations reached
      * Time budget exceeded
      * User interruption
Examples of what to implement:
    - is_goal_reached(node_position, goal_position, threshold)
    - track best node found so far
    - termination condition checks in main loop

TODO 9: PATH EXTRACTION & RECONSTRUCTION
--------
Purpose: Convert tree structure into an actual path (sequence of states)
Details:
    - Given a goal node, backtrack to root following parent pointers
    - Reverse the path (goes backward from goal → start)
    - May want to smooth/simplify path as post-processing
    - Return as list of states or list of (x, y) positions
    - Track total path cost
Examples of what to implement:
    - extract_path(tree, goal_node)
    - backtracking with parent pointers
    - path smoothing [future enhancement]
    - path cost accumulation

TODO 10: MAIN RRT PLANNING LOOP
--------
Purpose: Orchestrate all components to build tree and find path
Details:
    - Initialize tree with start configuration
    - Main loop:
        FOR each iteration (up to max_iter or until goal found):
            1. Sample random config
            2. Find nearest node
            3. Steer toward sample
            4. Collision check new edge
            5. If free, add to tree
            6. Check goal
            7. [RRT*] Rewire neighbors
    - Return best path found
Examples of what to implement:
    - plan(start, goal, max_iterations, max_time, step_size, etc.)
    - iteration loop with progress tracking
    - integration of all helper functions
    - logging/visualization hooks

TODO 11: TREE DATA STRUCTURE
--------
Purpose: Efficient storage and traversal of all nodes in the RRT
Details:
    - Node object: stores position, parent, cost, children list (for RRT*)
    - Tree object: root node, all nodes list, spatial index (KD-tree future)
    - Support operations: add_node, find_neighbors, get_path
    - Optional: visualization data
Examples of what to implement:
    - Node class with parent, cost, position attributes
    - Tree class with add/search/query methods
    - Optional spatial index for fast neighbor queries

TODO 12: DYNAMICS MODULE INTEGRATION (PLUGGABLE)
--------
Purpose: Allow dynamics team to plug in robot behavior without rewriting RRT
Details:
    - RobotDynamics interface/base class
    - Methods dynamics module must provide:
        * move_cost(state1, state2) -> float
        * trajectory(state1, state2, step_size) -> State
        * get_max_speed() -> float
    - Placeholder implementation for now (linear interpolation, constant speed)
    - Path planning passes dynamics_model to collision checking and steering
Examples of what to implement:
    - RobotDynamics base class with interface
    - Placeholder implementations
    - Integration points: steer(), collision_check(), edge_cost()

TODO 13: MAIN RRT* PLANNING LOOP
--------------------------------
Purpose: Build an optimized tree (not just feasible) using rewiring
Details:
    - Initialize tree with start configuration (cost = 0)
    - Main loop:
        FOR each iteration (up to max_iter or until goal found):
            1. Sample random config
               - With goal biasing (optional)
            2. Find nearest node in tree
            3. Steer toward sample (using Dubins / dynamics)
            4. Collision check new edge
               - Must be collision-free AND within bounds
            5. Find neighbors within radius r
               - r = γ * (log(n) / n)^(1/d)
               - n = number of nodes
               - d = dimension (2 for x,y)
            6. Choose BEST PARENT (minimum cost)
               FOR each neighbor:
                   - Try connecting neighbor → new node
                   - Check collision + bounds
                   - Compute cost:
                        cost = neighbor.cost + edge_cost(neighbor, new)
                   - Pick lowest-cost valid parent
            7. Add new node to tree
               - Assign best parent
               - Store cost
            8. REWIRE neighbors (optimization step)
               FOR each neighbor:
                   - Try connecting new node → neighbor
                   - Check collision + bounds
                   - Compute new cost:
                        cost = new_node.cost + edge_cost(new, neighbor)
                   - If cheaper:
                        update neighbor.parent
                        update neighbor.cost
                        update path
            9. Check if goal reached
               - Distance threshold
            10. If goal reached:
                - Store best goal node
                - Continue for a few iterations (optional) to improve cost
                - Or terminate early
    - After loop:
        - If goal found:
            backtrack from goal → start using parent pointers
            return path
        - Else:
            return None (no path found)
            
TODO 14: MAIN RRT*-FND PLANNING LOOP
--------------------------------
Purpose:
    Maintain and repair an RRT* path in the presence of dynamic obstacles
    without rebuilding the entire tree from scratch
Details:
    - Phase 1: INITIAL PLANNING
        - Run RRT* (or RRT*FN) to compute path from start → goal
        - Store:
            τ = full tree
            σ = solution path (list of nodes)
    - Phase 2: EXECUTION + MONITORING
        - Set current node:
            p_current = start
        - While p_current is not goal:
            1. Move along current path σ
            2. Update dynamic obstacles
                - Environment changes over time
            3. Check if path ahead is still valid
                - DetectCollision(σ, p_current) --> checks all future edges in the path for collisions with update obstacles
            4. IF collision detected:
                a. Stop movement
                b. Split path (SelectBranch function):
                    - Parent tree τ at p_current
                    - Future branch (potentially invalid)
                c. ValidPath function:
                    - Removes nodes/edges that collide with obstacles
                    - The removed portion becomes σ_separate
                d. Try RECONNECT:
                    - Find nearby nodes in main tree τ
                    - Attempt direct connection:
                        ObstacleFree(p_near, σ_separate)
                    - If successful:
                        τ ← reconnect trees
                e. If reconnect fails:
                    - REGROW:
                        Grow tree τ from p_current
                        Bias toward σ_separate region
                f. Recompute solution path:
                    σ ← SolutionPath(τ, p_current)
                g. Resume movement
            5. Move to next node:
                p_current ← next node in σ
    - End when goal reached
================================================================================
"""

from typing import List, Tuple, Optional
import math
import time
import random
import numpy as np
import matplotlib.pyplot as plt
from dynamics import State

# TODO: Import dynamics definitions from your dynamics module


# ============================================================================
# TODO 11: TREE DATA STRUCTURE
# ============================================================================

class TreeNode:
    """Represents a single node in the RRT tree."""
    
    def __init__(self, state: State, parent=None, cost: float = 0.0):
        """
        Args:
            state: Configuration at this node
            parent: Parent node in tree
            cost: Cost from start to this node (cumulative)
        """
        self.state = state
        self.parent = parent
        self.cost = cost  # Total cost from start to here
        self.children = []
    
    def __repr__(self):
        return f"TreeNode(state={self.state}, cost={self.cost:.2f})"


class RRTTree:
    """Manages the RRT tree structure."""
    
    def __init__(self, start: State):
        """
        Args:
            start: Root node configuration
        """
        self.root = TreeNode(start, parent=None, cost=0.0)
        self.nodes = [self.root]
    
    def add_node(self, state: State, parent: TreeNode, cost: float) -> TreeNode:
        """
        Add a new node to the tree.
        
        Args:
            state: New node configuration
            parent: Parent node
            cost: Cost from start to this new node
        
        Returns:
            The newly created node
        """
        new_node = TreeNode(state, parent=parent, cost=cost)
        parent.children.append(new_node)
        self.nodes.append(new_node)
        return new_node
    
    def size(self) -> int:
        """Return number of nodes in tree."""
        return len(self.nodes)
    
    def get_nearest_node(self, state: State) -> TreeNode:
        """
        Find nearest node to given state.
        
        Args:
            state: Query state
        
        Returns:
            Nearest node in tree
        """
        nearest = self.root
        min_dist = State.state_distance(state, nearest.state)
        
        for node in self.nodes[1:]:
            dist = State.state_distance(state, node.state)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return nearest
    
    def get_neighbors_in_radius(self, state: State, radius: float) -> List[TreeNode]:
        """
        Find all nodes within radius of given state.
        
        Args:
            state: Query state
            radius: Search radius
        
        Returns:
            List of nodes within radius
        """
        neighbors = []
        for node in self.nodes:
            if State.state_distance(state, node.state) <= radius:
                neighbors.append(node)
        return neighbors


# ============================================================================
# TODO 2: RANDOM SAMPLING
# ============================================================================

def sample_random_state(map_bounds: Tuple, goal: State = None, 
                         p_goal: float = 0.05) -> State:
    """
    Sample a random configuration.
    With probability p_goal, return the goal. Otherwise, sample uniformly.
    
    Args:
        map_bounds: Tuple of (x_min, x_max, y_min, y_max)
        goal: Goal state (if None, never sample goal)
        p_goal: Probability of sampling goal directly
    
    Returns:
        Randomly sampled state
    """
    if goal is not None and random.random() < p_goal:
        return goal
    
    # Random uniform in map bounds
    x = random.uniform(map_bounds[0], map_bounds[1])
    y = random.uniform(map_bounds[2], map_bounds[3])
    theta = random.uniform(0, 2 * math.pi)
    return State(x, y, theta)


# ============================================================================
# TODO 3: NEAREST NEIGHBOR SEARCH
# ============================================================================

def nearest_node(tree: RRTTree, config: State) -> TreeNode:
    """
    Find the nearest node in the tree to the given configuration.
    
    Args:
        tree: RRT tree
        config: Query configuration
    
    Returns:
        Nearest TreeNode
    """
    return tree.get_nearest_node(config)


# ============================================================================
# TODO 4: STEERING / LOCAL EDGE INTERPOLATION
# ============================================================================

'''
def steer(start: State, target: State, step_size: float, dynamics_model):
    """
    Steer from start toward target using bounded step size.
    Returns a small trajectory (Nx3 array).
    """

    dx = target.x - start.x
    dy = target.y - start.y

    dist = math.sqrt(dx*dx + dy*dy)

    if dist == 0:
        return None

    # Limit movement to step_size
    step = min(step_size, dist)

    # Unit direction
    ux = dx / dist
    uy = dy / dist

    # New target point (bounded)
    new_x = start.x + step * ux
    new_y = start.y + step * uy

    # Orientation toward motion direction
    new_theta = math.atan2(uy, ux)

    # Create simple straight-line trajectory
    num_points = 10
    trajectory = np.zeros((num_points, 3))

    for i in range(num_points):
        t = (i + 1) / num_points
        x = start.x + t * (new_x - start.x)
        y = start.y + t * (new_y - start.y)
        theta = new_theta

        trajectory[i] = [x, y, theta]

    return trajectory
'''

def steer(start: State, target: State, step_size: float, dynamics_model):
    """
    Steer from start toward target, moving at most step_size distance.
    Uses dynamics model for robot-specific behavior.
    
    Args:
        start: Starting state
        target: Target state to steer toward
        step_size: Maximum distance to travel
        dynamics_model: Robot dynamics (has trajectory() method)
    
    Returns:
        New state after steering, or None if steering fails
    """
    # TODO: Implement steering logic
    # For now, use dynamics model's trajectory method
    if dynamics_model is None:
        return None
    
    # STEP 1: compute control
    v, omega = dynamics_model.robot_controller(start, target)

    # STEP 2: simulate trajectory
    trajectory = dynamics_model.trajectory(start, v, omega)

    if len(trajectory) == 0:
        return None
    
    # Proper truncation (distance-based)
    truncated = []
    total_dist = 0.0

    prev_x, prev_y = start.x, start.y

    for x, y, theta in trajectory:
        dx = x - prev_x
        dy = y - prev_y
        d = math.sqrt(dx*dx + dy*dy)

        total_dist += d
        truncated.append([x, y, theta])

        if total_dist >= step_size:
            break

        prev_x, prev_y = x, y

    return np.array(truncated)

# ============================================================================
# TODO 5: COLLISION CHECKING
# ============================================================================

def inflate_rectangle(rect, radius):
    """
    Inflate rectangle by robot radius.

    Args:
        rect: (x, y, w, h)
        radius: robot radius

    Returns:
        inflated rectangle (x, y, w, h)
    """
    rx, ry, w, h = rect

    return (
        rx - radius,
        ry - radius,
        w + 2 * radius,
        h + 2 * radius
    )

def point_in_rectangle(point: Tuple[float, float], rect: Tuple[float, float, float, float]):
    """
    Check if a point is inside an axis-aligned rectangle.

    Args:
        point: (x, y)
        rect: (x_min, y_min, width, height)

    Returns:
        True if inside rectangle, False otherwise
    """
    px, py = point
    rx, ry, w, h = rect

    return (rx <= px <= rx + w) and (ry <= py <= ry + h)

def is_collision_free_trajectory(trajectory, dynamics_model, t_start=0.0):
    """
    Check if path from start to end is collision-free.
    Accounts for static obstacles and optionally dynamic obstacles.
    
    Args:
        start: Starting state
        end: Ending state
        static_obstacles: List of static obstacles (rectangles)
        dynamics_model: Robot dynamics (optional, for dynamic obstacles)
        t_start: Starting time for collision check
    
    Returns:
        True if path is collision-free, False otherwise
    """
    # TODO: Implement collision checking

    radius = dynamics_model.robot_radius

    if len(trajectory) == 0:
        return False
    dt = 1.0 / len(trajectory)

    for i, point in enumerate(trajectory):
        x, y, _ = point

        t = t_start + i*dt

        # print(f"full static obstacles = {dynamics_model.static_obstacles}")

        # 1. Static obstacles check
        for obs in dynamics_model.static_obstacles:
            inflated_obs = inflate_rectangle(obs, radius)
            if point_in_rectangle((x, y), inflated_obs):
                return False
            
        # 2. Dynamic obstacles check
        for obs_id in range(len(dynamics_model.dynamic_obstacles)):
            obs_rect = dynamics_model.obstacle_position_at_time(obs_id, t)

            inflated_obs = inflate_rectangle(obs_rect, radius)
            if point_in_rectangle((x, y), inflated_obs):
                return False        

    return True


# ============================================================================
# TODO 6: EDGE COST COMPUTATION
# ============================================================================

def edge_cost(s1: State, s2: State, dynamics_model=None) -> float:
    """
    Compute cost (time/distance) to traverse from s1 to s2.
    
    Args:
        s1: Starting state
        s2: Ending state
        dynamics_model: Robot dynamics (optional, for speed-based costs)
    
    Returns:
        Cost value
    """
    # TODO: Implement cost computation
    # For now, use Euclidean distance
    return State.state_distance(s1, s2)


# ============================================================================
# TODO 7: REWIRING (RRT* - FUTURE)
# ============================================================================

def rewire_neighbors(tree: RRTTree, new_node: TreeNode, radius: float, dynamics_model=None):
    """
    RRT* rewiring: optimize tree by rerouting through cheaper paths.
    
    Args:
        tree: RRT tree
        new_node: Newly added node to rewire around
        radius: Search radius for neighbors
        dynamics_model: Robot dynamics (for cost computation)
    """
    # TODO: Implement RRT* rewiring
    pass


# ============================================================================
# TODO 8: GOAL DETECTION & TERMINATION
# ============================================================================

def is_goal_reached(node_state: State, goal: State, threshold: float = 0.1) -> bool:
    """
    Check if node is within threshold distance of goal.
    
    Args:
        node_state: Node state to check
        goal: Goal state
        threshold: Distance threshold
    
    Returns:
        True if node is close enough to goal
    """
    return State.state_distance(node_state, goal) < threshold


# ============================================================================
# TODO 9: PATH EXTRACTION & RECONSTRUCTION
# ============================================================================

def extract_path(goal_node: TreeNode) -> List[State]:
    """
    Extract path by backtracking from goal node to root.
    
    Args:
        goal_node: Goal TreeNode to extract path from
    
    Returns:
        Path as list of states from start to goal
    """
    path = []
    node = goal_node
    
    while node is not None:
        path.append(node.state)
        node = node.parent
    
    path.reverse()
    return path


# ============================================================================
# TODO 10: MAIN RRT PLANNING LOOP
# ============================================================================

def visualize_rrt(ax, tree, path=None):
    """
    Draw the RRT tree and optional final path.
    """
    # Draw tree edges
    for node in tree.nodes:
        if node.parent is not None:
            x1, y1 = node.state.x, node.state.y
            x2, y2 = node.parent.state.x, node.parent.state.y
            ax.plot([x1, x2], [y1, y2], color='blue', linewidth=0.5)

    # Draw final path if exists
    if path is not None:
        xs = [s.x for s in path]
        ys = [s.y for s in path]
        ax.plot(xs, ys, color='yellow', linewidth=2, label='Path')

def plan_rrt(start, goal, map_info, dynamics_model, ax=None, max_iterations=5000, max_time=60.0, step_size=1.0, goal_threshold=0.1, p_goal_bias=0.05):
    """
    Main RRT planning function.
    
    Args:
        start: Starting configuration
        goal: Goal configuration
        map_info: Map with obstacles and bounds
            Required keys: 'bounds' (x_min, x_max, y_min, y_max), 'obstacles'
        dynamics_model: Robot dynamics (pluggable)
        max_iterations: Maximum planning iterations
        max_time: Time budget (seconds)
        step_size: Maximum extension distance per iteration
        goal_threshold: Distance threshold to consider goal reached
        p_goal_bias: Probability of sampling goal directly (0.0 to 1.0)
    
    Returns:
        Path as list of states, or None if no path found
    """
    start_time = time.time()
    iterations = 0
    tree = RRTTree(start)
    
    # Extract map info
    rows, cols = map_info.dimensions
    map_bounds = (0, cols, 0, rows)

    while iterations < max_iterations:
        # Check time budget
        if time.time() - start_time > max_time:
            break
        
        # 1. Sample random config
        q_rand = sample_random_state(map_bounds, goal=goal, p_goal=p_goal_bias)
        
        # 2. Find nearest node
        q_nearest_node = nearest_node(tree, q_rand)
        
        # 3. Steer toward sample
        # trajectory = steer(q_nearest_node.state, q_rand, dynamics_model)
        trajectory = steer(q_nearest_node.state, q_rand, step_size, dynamics_model)
        
        if trajectory is None:
            iterations += 1
            continue
        
        # 4. Collision check
        if is_collision_free_trajectory(trajectory, dynamics_model, t_start=0.0):

            # 5. Extract last node of the trajectory as a State
            x, y, theta = trajectory[-1]
            q_new = State(x, y, theta)            

            # 6. Add to RRT tree
            new_cost = q_nearest_node.cost + edge_cost(q_nearest_node.state, q_new, dynamics_model)
            new_node = tree.add_node(q_new, parent=q_nearest_node, cost=new_cost)
            
            # =========================
            # ALWAYS VISUALIZE
            # =========================
            if ax is not None and iterations % 20 == 0:
                if iterations % 100 == 0:
                    print(f"Length of tree is: {tree.size()}")
                ax.clear()

                grid = map_info.grid
                ax.imshow(1 - grid, cmap='gray', origin='upper')

                visualize_rrt(ax, tree)

                ax.scatter(start.x, start.y, c='green', s=100)
                ax.scatter(goal.x, goal.y, c='red', s=100)

                ax.set_title(f"RRT Growth (iter={iterations})")
                ax.set_xlim(0, grid.shape[1])
                ax.set_ylim(grid.shape[0], 0)

                plt.pause(0.01)

            # 7. Check if reached point is at goal point
            if is_goal_reached(q_new, goal, goal_threshold):
                path = extract_path(new_node)
                planning_time = time.time() - start_time
                print(f"Path found in {planning_time:.2f}s, {iterations} iterations, path length: {len(path)}")
                return path
        
        iterations += 1
    
    # No path found
    planning_time = time.time() - start_time
    print(f"No path found after {planning_time:.2f}s, {iterations} iterations")
    return None

# ============================================================================
# HELPER FUNCTION --> Grid to obstacle conversion (for collision checking)
# ============================================================================
def grid_to_obstacles(grid):
    obstacles = []
    rows, cols = grid.shape

    for r in range(rows):
        for c in range(cols):
            if grid[r, c] == 1:
                obstacles.append((c, r, 1, 1))  # (x, y, w, h)

    return obstacles

# ============================================================================
# TODO 13: MAIN RRT* PLANNING LOOP
# ============================================================================
def plan_rrt_star(start: State, goal: State, map_info, dynamics_model, 
                  max_iterations: int = 5000, max_time: float = 10.0, 
                  step_size: float = 1.0, goal_threshold: float = 0.5,
                  p_goal_bias: float = 0.05) -> Optional[List[State]]:

    start_time = time.time()
    iterations = 0
    tree = RRTTree(start) # initialize tree with start node

    # Extract map info
    rows, cols = map_info.dimensions
    map_bounds = (0, cols, 0, rows)

    while iterations < max_iterations:
        # Time check
        if time.time() - start_time > max_time:
            break

        # Step 1: Sample random configuration (with goal biasing))
        q_rand = sample_random_state(map_bounds, goal = goal, p_goal = p_goal_bias)

        # Step 2: Find nearest node in tree
        q_nearest_node = nearest_node(tree, q_rand)

        # Step 3: Steer from nearest node toward random sample (won't necessarily reach q_rand, just move step_size in that direction)
        # trajectory = steer(q_nearest_node.state, q_rand, dynamics_model)
        trajectory = steer(q_nearest_node.state, q_rand, step_size, dynamics_model)

        if trajectory is None: # if steering fails (e.g., due to dynamics constraints), skip this iteration
            iterations += 1
            continue

        # Step 4: Collision check the new edge (considering static + dynamic obstacles) -> checks along trajectory not just endpoint
        if is_collision_free_trajectory(trajectory, dynamics_model, t_start=0.0):

            x, y, theta = trajectory[-1]
            q_new = State(x, y, theta) # taking the last point in that trajectory as the new node position

            # Step 5: Find neighbors within radius defined by formula r = gamma * (log(n) / n)^(1/d)
            n = tree.size()
            d = 2
            gamma = 5.0  # tuning parameter

            radius = gamma * (math.log(n + 1) / (n + 1)) ** (1/d) # radius shrinks as tree grows (explore widely at the start, then refine locally later on)

            neighbors = tree.get_neighbors_in_radius(q_new, radius)

            # Step 6: Choose best parent (minimum cost) from neighbors (start with nearest node as default parent)
            best_parent = q_nearest_node
            best_cost = q_nearest_node.cost + edge_cost(q_nearest_node.state, q_new, dynamics_model)

            for neighbor in neighbors:

                # traj = steer(neighbor.state, q_new, dynamics_model) # try connecting neighbor to new node
                traj = steer(neighbor.state, q_new, step_size, dynamics_model)
                if traj is None: # steering failure check
                    continue

                if not is_collision_free_trajectory(traj, dynamics_model, t_start=0.0): # collision check
                    continue

                cost = neighbor.cost + edge_cost(neighbor.state, q_new, dynamics_model) # cost of being at neighbor + cost to get to new node

                if cost < best_cost:
                    best_parent = neighbor
                    best_cost = cost

            # Step 7: Add new node to tree with best parent and cost
            new_node = tree.add_node(q_new, parent = best_parent, cost = best_cost)

            # Step 8: Rewire neighbors (optimization step) -> check if going through new node is cheaper for any of the neighbors
            for neighbor in neighbors:

                # traj = steer(q_new, neighbor.state, dynamics_model) # try connecting new node to neighbor (reverse direction from before; new node is now the parent)
                traj = steer(q_new, neighbor.state, step_size, dynamics_model)
                if traj is None:
                    continue

                if not is_collision_free_trajectory(traj, dynamics_model, t_start=0.0):
                    continue

                new_cost = new_node.cost + edge_cost(q_new, neighbor.state, dynamics_model)

                if new_cost < neighbor.cost:
                    neighbor.parent = new_node # rewire neighbor to new node
                    neighbor.cost = new_cost

            # Step 9: Check if goal reached
            if is_goal_reached(q_new, goal, goal_threshold):
                path = extract_path(new_node)
                planning_time = time.time() - start_time
                print(f"[RRT*] Path found in {planning_time:.2f}s, {iterations} iterations, length: {len(path)}")
                return path

        iterations += 1

    print("[RRT*] No path found")
    return None

# ============================================================================
# TODO 14: MAIN RRT*FND PLANNING LOOP
# ============================================================================
def plan_rrt_star_fnd(start: State, goal: State, map_info, dynamics_model,
                      max_iterations: int = 5000, max_time: float = 10.0,
                      step_size: float = 1.0, goal_threshold: float = 0.5,
                      p_goal_bias: float = 0.05):
    
    # Step 1: Initial Planning Phase (same as RRT*)
    # - run the existing RRT* planner to compute an initial path from start to goal
    # - build the full tree (τ) and extract the solution path (σ)
    # - this will be the baseline path before any dynamic obstacles are introduced
    
    # Step 2: Initialization
    # - set the current node (p_current) to the start configuration
    # - keep track of current path (σ) 
    # - initialize any time or iteration tracking if needed for monitoring
    
    # Step 3: Main Execution Loop
    # - while p_current is not the goal
    
        # Step 3.1: Move along the current path σ
        # - move the robot forward along the path σ, updating p_current as it progresses
        # - this simulates the robot executing the planned path in the environment
        
        # Step 3.2: Update dynamic obstacles
        # - update positions of moving obstacles (should be provided by dynamics module or environment model)
        # - environment now changes over time, so we need to keep track of where obstacles are at the current time step
        
        # Step 3.3: Collision Detection
        # - check if the path ahead (the remaining portion of σ from p_current to goal) is still valid given the updated obstacle positions
        # - this involves checking all future edges in the path for collisions with the new obstacle positions
        # - if a collision is detected, we need to trigger the path repair process
        
            # IF COLLISION IS DETECTED
            # Step 3.3.1: Stop Movement
            # - immediately halt the robot's movement to prevent collision
            # - robot stays at p_current, which is the last safe position before the collision
            
            # Step 3.3.2: SelectBranch function
            # - Split the tree τ at p_current
            # - Keep: nodes still connected to p_current (main tree)
            # - Separate: forward portion of path toward goal
            
            # Step 3.3.3: ValidPath function
            # - Remove nodes/edges that collide with obstacles
            # - This cleans the tree
            # - The removed portion becomes σ_separate (from node right after obstacle to goal)
            
            # Step 3.3.4: Try Reconnect
            # - Find nearby nodes in main tree τ that are close to the start of σ_separate
            # - Attempt direct connection
            # - For each nearby node, check if connecting to σ_separate is collision-free and is a valid trajectory
            # - If successful, reconnect the trees (main tree + separate path) and update σ and break loop
            
            # Step 3.3.5: If Reconnect Fails, Regrow
            # - If no valid connection is found, we need to regrow the tree τ from p_current
            # - Bias the growth toward the region of σ_separate to try to find a new path around the obstacle
            # - This involves running a new RRT* from p_current with the same goal, but with updated obstacle information
            
            # Step 3.3.6: Recompute Solution Path
            # - After reconnecting or regrowing, we need to recompute the solution path σ from p_current to goal using the updated tree τ
            # - This will give us a new path that avoids the newly detected obstacle
            
            # Step 3.3.7: Resume Movement
            # - Once we have a new path σ, we can resume movement along this path toward
            
        # Step 3.4: Move to Next Node
        # - Update p_current to the next node in the current path σ
        
    # Step 4: End when goal reached
    # - Once p_current is within the goal threshold, we can terminate and return the final path taken to reach the goal
            
    return None # placeholder
    
    
# ============================================================================
# EXAMPLE USAGE
# ============================================================================

# if __name__ == "__main__":
    # print("=" * 70)
    # print("RRT PLANNER EXAMPLE")
    # print("=" * 70)
    
    # # Setup map info
    # map_info = {
    #     'bounds': (0, 100, 0, 100),
    #     'obstacles': [],
    # }
    
    # # Setup start and goal
    # start = State(10, 10, 0)
    # goal = State(90, 90, 0)
    
    # print(f"\nStart: {start}")
    # print(f"Goal: {goal}")
    
    # # Plan without dynamics (fallback steering)
    # path = plan_rrt(start, goal, map_info, dynamics_model=None, 
    #                max_iterations=1000, max_time=5.0, step_size=2.0)
    
    # if path:
    #     print(f"\nFound path with {len(path)} waypoints:")
    #     for i, state in enumerate(path):
    #         print(f"  {i}: {state}")
    # else:
    #     print("\nNo path found")