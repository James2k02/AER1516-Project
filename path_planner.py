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
from dynamics import State
from utils import update_obstacles

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
        if total_dist > step_size:
            break
        
        truncated.append([x, y, theta])

        prev_x, prev_y = x, y

    return np.array(truncated)

def steer_full(start: State, target: State, dynamics_model):
    v, omega = dynamics_model.robot_controller(start, target)
    trajectory = dynamics_model.trajectory(start, v, omega)

    return trajectory  # NO truncation

# ============================================================================
# TODO 5: COLLISION CHECKING
# ============================================================================

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

    if trajectory is None or len(trajectory) == 0:
        return False

    radius = dynamics_model.robot_radius
    dt = 1.0 / len(trajectory)

    for i, point in enumerate(trajectory):
        x, y, _ = point

        t = t_start + i * dt

        # 1. Static obstacles check
        for obs in dynamics_model.static_obstacles:
            if obs.collides_with_point(x, y, radius):
                return False
        
        '''
        # 2. Dynamic obstacles check
        for obs in dynamics_model.dynamic_obstacles:
            obs_t = obs.get_position_at_time(t, dynamics_model.grid)
            if obs_t.collides_with_point(x, y, radius):
                return False
        '''
        
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
#  REWIRING (RRT*)
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
    # Find all neighbors within radius of new_node
    neighbors = tree.get_neighbors_in_radius(new_node.state, radius)

    for neighbor in neighbors:
        # Skip the new node itself
        if neighbor == new_node:
            continue

        # Calculate cost if we rewire through new_node
        edge_cost_value = edge_cost(new_node.state, neighbor.state, dynamics_model)
        new_path_cost = new_node.cost + edge_cost_value

        # Only consider rewiring if path is cheaper
        if new_path_cost < neighbor.cost:
            # Check if path from new_node to neighbor is collision-free
            trajectory = steer_full(new_node.state, neighbor.state, dynamics_model)
            if is_collision_free_trajectory(trajectory, dynamics_model):
                # Rewire: update parent and cost
                neighbor.parent.children.remove(neighbor)
                new_node.children.append(neighbor)
                neighbor.parent = new_node
                neighbor.cost = new_path_cost

                # Propagate cost changes to all descendants
                propagate_cost(neighbor)


def propagate_cost(node: TreeNode):
    """
    Recursively update costs of all descendants after a rewire.
    Must be called after changing node.cost so children inherit the delta.
    """
    for child in node.children:
        child.cost = node.cost + edge_cost(node.state, child.state)
        propagate_cost(child)


# ============================================================================
#  GOAL DETECTION & TERMINATION
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
# PATH EXTRACTION & RECONSTRUCTION
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
# MAIN RRT PLANNING LOOP
# ============================================================================

def plan_rrt(start, goal, map_info, dynamics_model, max_iterations=5000, max_time=60.0, step_size=1.0, goal_threshold=0.1, p_goal_bias=0.05, viz_callback=None, viz_interval: int = 20):
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

            # 7. Check if reached point is at goal point
            if is_goal_reached(q_new, goal, goal_threshold):
                path = extract_path(new_node)
                planning_time = time.time() - start_time
                print(f"Path found in {planning_time:.2f}s, {iterations} iterations, path length: {len(path)}")
                return path

        if viz_callback is not None and iterations % viz_interval == 0:
            viz_callback({"planner": "RRT", "tree": tree, "iteration": iterations, "phase": "planning"})

        iterations += 1

    # No path found
    planning_time = time.time() - start_time
    print(f"No path found after {planning_time:.2f}s, {iterations} iterations")
    return None

# ============================================================================
# MAIN RRT* PLANNING LOOP
# ============================================================================
def plan_rrt_star(start: State, goal: State, map_info, dynamics_model, max_iterations: int = 20000, max_time: float = 60.0, step_size: float = 1.0, goal_threshold: float = 0.1, p_goal_bias: float = 0.05, viz_callback=None, viz_interval: int = 20, tree = None):

    start_time = time.time()
    iterations = 0
    if tree is None:
        tree = RRTTree(start) # initialize tree with start node

    # Extract map info
    rows, cols = map_info.dimensions
    map_bounds = (0, cols, 0, rows)

    goal_reached = False
    goal_node = None

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
            gamma = 15.0  # tuning parameter — theoretically gamma_star ≈ 25 for a 20x20 2D map

            radius = gamma * (math.log(n + 1) / (n + 1)) ** (1/d) # radius shrinks as tree grows (explore widely at the start, then refine locally later on)

            neighbors = tree.get_neighbors_in_radius(q_new, radius)

            # Step 6: Choose best parent (minimum cost) from neighbors (start with nearest node as default parent)
            best_parent = q_nearest_node
            best_cost = q_nearest_node.cost + edge_cost(q_nearest_node.state, q_new, dynamics_model)

            for neighbor in neighbors:

                # traj = steer(neighbor.state, q_new, dynamics_model) # try connecting neighbor to new node
                traj = steer_full(neighbor.state, q_new, dynamics_model)  # no cap: must reach q_new
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

            # Step 8: Rewire neighbors (optimization step)
            rewire_neighbors(tree, new_node, radius, dynamics_model)

            # Step 9: Check if goal reached — keep cheapest goal node found
            if is_goal_reached(q_new, goal, goal_threshold):
                goal_reached = True
                if goal_node is None or new_node.cost < goal_node.cost:
                    goal_node = new_node

        if viz_callback is not None and iterations % viz_interval == 0:
            viz_callback({"planner": "RRT*", "tree": tree, "iteration": iterations, "phase": "planning"})

        iterations += 1

    if goal_reached:
        path = extract_path(goal_node)
        planning_time = time.time() - start_time
        print(f"[RRT*] Path found in {planning_time:.2f}s, {iterations} iterations, length: {len(path)}")
        return path, tree

    else:
        print("[RRT*] No path found")
        return None, tree
    

# ============================================================================
# FUNCTIONS FOR RRT*FND
# ============================================================================
def detect_future_collision(path: List[State], current_node: int, dynamics_model, t_current: float = 0.0):
    """
    Check if any future edge in the path from current_node
    to goal collides with updated dynamic obstacles.

    t_current: the global time (in obstacle timesteps * dt=0.1) at which the robot
               is currently at current_node. Each future edge i is checked at
               t_start = t_current + (i - current_node) * 0.1, so obstacle positions
               are predicted consistently with how many update_obstacles() calls have run.
    """
    for i in range(current_node, len(path) - 1):
        traj = steer_full(path[i], path[i+1], dynamics_model = dynamics_model)

        # return True if trajectory is not valid (either steering failure or collision)
        if traj is None:
            return True

        # Each path step corresponds to one obstacle update (dt=0.1 in get_position_at_time)
        t_start = t_current + (i - current_node) * 0.1
        if not is_collision_free_trajectory(traj, dynamics_model, t_start=t_start):
            return True
    return False

def select_branch(path: List[State], current_index: int):
    """
    Split path into:
    - Main branch: nodes still connected to current position (up to current_index)
    - Separate branch: forward portion of path toward goal (from current_index + 1 to end)
    """
    sigma_main = path[:current_index + 1] # from start to current position including current node 
    sigma_separate = path[current_index + 1:] # future portion of path toward goal
    
    return sigma_main, sigma_separate

def valid_path(sigma_separate: List[State], dynamics_model):
    """
    Removes any invalid nodes/edges from the path that collide with obstacles, returning the cleaned path
    """
    valid = [sigma_separate[0]]
    for i in range(len(sigma_separate) - 1):
        traj = steer(sigma_separate[i], sigma_separate[i+1], 1.0, dynamics_model)
        
        if traj is None:
            break
        
        if not is_collision_free_trajectory(traj, dynamics_model):
            break
    
        valid.append(sigma_separate[i+1]) 
    return valid

def attach_branch(tree: RRTTree, start_node: List[State], sigma_separate: List[State], dynamics_model):
    """
    Attach the rest of sigma_separate back to the main tree at the new connection point (new_node)
    """
    prev = start_node
    
    # here, we are starting from the node after the connection point since the connection point itself is already in the tree (as start_node)
    for i in range(1, len(sigma_separate)):
        s = sigma_separate[i]
        new_node = tree.add_node(s, parent = prev, cost = prev.cost + edge_cost(prev.state, s, dynamics_model))
        prev = new_node
    return prev # return the last node added (which should be the goal node if sigma_separate is valid all the way to the end)

def reconnect(tree: RRTTree, sigma_separate: List[State], dynamics_model):
    """
    Try to reconnect the separate branch back to the main tree by finding nearby nodes in the main tree and 
    checking for collision-free connections to the start of the separate branch.  If successful, update the 
    tree structure and return True. If no valid connection is found, return False.
    """
    if len(sigma_separate) == 0:
        return None
    
    # Try to connect the main tree into any of the nodes in the separate branch (starting from the first node in separate branch)
    for target in sigma_separate:
        
        # Loop through nodes in the main tree to find nearby nodes to target
        for node in tree.nodes:
            traj = steer(node.state, target, step_size = 1.0, dynamics_model = dynamics_model)
            
            if traj is None:
                continue
            
            if is_collision_free_trajectory(traj, dynamics_model):
                new_node = tree.add_node(target, parent = node, cost = node.cost + edge_cost(node.state, target, dynamics_model))
                
                # attach the rest of the separate branch to this new node (node in the separate branch that we just connected to the main tree)
                final_node = attach_branch(tree, new_node, sigma_separate, dynamics_model)
                return final_node # return the final node added (which should be the goal node if successful)
    return None

def regrow(tree, current_state, goal, map_info, dynamics_model):
    """
    If reconnect fails, we need to regrow the tree from the current position. 
    This involves running a new RRT* from the current state with the same goal, but with updated obstacle information.
    We can bias the growth toward the region of the separate branch to try to find a new path around the obstacle.
    """
    # For simplicity, we can just call plan_rrt_star with the current state as the new start and biasing toward the first node in sigma_separate
    # Increase goal bias to encourage finding a path toward goal quickly
    path, tree = plan_rrt_star(current_state, goal, map_info, dynamics_model, max_iterations=1000, p_goal_bias=0.3, tree=tree) # reuse tree
    return path 


# ============================================================================
# MAIN RRT*FND PLANNING LOOP
# ============================================================================
def plan_rrt_star_fnd(start: State, goal: State, map_info, dynamics_model,
                      max_iterations: int = 20000, max_time: float = 60.0,
                      step_size: float = 1.0, goal_threshold: float = 0.5,
                      p_goal_bias: float = 0.05, viz_callback=None, 
                      viz_interval: int = 20):
    
    # Step 1: Initial Planning Phase (same as RRT*)
    # - run the existing RRT* planner to compute an initial path from start to goal
    # - build the full tree (τ) and extract the solution path (σ)
    # - this will be the baseline path before any dynamic obstacles are introduced
    path, tree = plan_rrt_star(start, goal, map_info, dynamics_model,
                         max_iterations=max_iterations,
                         max_time=max_time,
                         step_size=step_size,
                         goal_threshold=goal_threshold,
                         p_goal_bias=p_goal_bias)
    
    if path is None:
        print("Initial planning failed, cannot execute RRT*FND")
        return None
    
    # Step 2: Initialization
    # - set the current node (p_current) to the start configuration
    # - keep track of current path (σ) 
    # - initialize any time or iteration tracking if needed for monitoring
    p_current = start
    current_index = 0
    repair_count = 0
    executed_path = [start]
    
    # Step 3: Main Execution Loop
    # - while p_current is not the goal
    while not is_goal_reached(p_current, goal, goal_threshold):
    
        # Step 3.1: Move along the current path σ
        # - move the robot forward along the path σ, updating p_current as it progresses
        # - this simulates the robot executing the planned path in the environment
        if current_index < len(path) - 1:
            current_index += 1
            p_current = path[current_index]
            executed_path.append(p_current)
        else:
            break
        
        # Step 3.2: Update dynamic obstacles
        # - update positions of moving obstacles (should be provided by dynamics module or environment model)
        # - environment now changes over time, so we need to keep track of where obstacles are at the current time step
        update_obstacles(dynamics_model.dynamic_obstacles, map_info.grid)
        
        # Step 3.3: Collision Detection
        # - check if the path ahead (the remaining portion of σ from p_current to goal) is still valid given the updated obstacle positions
        # - this involves checking all future edges in the path for collisions with the new obstacle positions
        # - if a collision is detected, we need to trigger the path repair process
        # t_current: each FND step calls update_obstacles once (one obstacle.update()),
        # and get_position_at_time uses dt=0.1 internally, so global time = current_index * 0.1
        t_current = current_index * 0.1
        if detect_future_collision(path, current_index, dynamics_model, t_current=t_current):
               
            # IF COLLISION IS DETECTED
            # Step 3.3.1: Stop Movement
            # - immediately halt the robot's movement to prevent collision
            # - robot stays at p_current, which is the last safe position before the collision
            print("🚨 COLLISION DETECTED — repairing path")
            repair_count += 1
            print(f"Repair triggered #{repair_count}")
            
            # Step 3.3.2: SelectBranch function
            # - Split the tree τ at p_current
            # - Keep: nodes still connected to p_current (main tree)
            # - Separate: forward portion of path toward goal
            sigma_main, sigma_separate = select_branch(path, current_index)
            
            # Step 3.3.3: ValidPath function
            # - Remove nodes/edges that collide with obstacles in the separate branch (the one that is potentially invalid due to the new obstacle)
            # - This cleans the tree
            # - The removed portion becomes σ_separate (from node right after obstacle to goal)
            sigma_separate = valid_path(sigma_separate, dynamics_model)
                    
            # Step 3.3.4: Try Reconnect
            # - Find nearby nodes in main tree τ that are close to the start of σ_separate
            # - Attempt direct connection
            # - For each nearby node, check if connecting to σ_separate is collision-free and is a valid trajectory
            # - If successful, reconnect the trees (main tree + separate path) and update σ and break looptree = RRTTree(p_current)
            # tree = RRTTree(p_current) --> treating current position as new root of the tree but this won't work because it's making a new tree
            final_node = reconnect(tree, sigma_separate, dynamics_model)
            print("Trying reconnect...")
            
            if final_node is not None:
                # new_path = extract_path(final_node)
                # path = sigma_main + new_path
                new_path = extract_path(final_node)

                current = sigma_main[-1]

                # find where current appears in new_path
                idx = None
                for i, s in enumerate(new_path):
                    if State.state_distance(s, current) < 1e-3:
                        idx = i
                        break

                if idx is not None:
                    new_path = new_path[idx:]
                    path = sigma_main + new_path[1:]
                else:
                    print("WARNING: reconnect path doesn't align, using fallback")
                    path = sigma_main + new_path
                    
                print("✅ Reconnected!")
            # Step 3.3.5: If Reconnect Fails, Regrow
            # - If no valid connection is found, we need to regrow the tree τ from p_current
            # - Bias the growth toward the region of σ_separate to try to find a new path around the obstacle
            # - This involves running a new RRT* from p_current with the same goal, but with updated obstacle information
            else:
                new_path = regrow(tree, p_current, goal, map_info, dynamics_model)

                if new_path is None:
                    return None

                current = sigma_main[-1]

                idx = None
                for i, s in enumerate(new_path):
                    if State.state_distance(s, current) < 1e-3:
                        idx = i
                        break

                if idx is not None:
                    new_path = new_path[idx:]
                    path = sigma_main + new_path[1:]
                else:
                    print("WARNING: regrow path doesn't align")
                    path = sigma_main + new_path
                    
                print("❌ Reconnect failed → Regrowing")
            
            # Step 3.3.6: Recompute Solution Path
            # - After reconnecting or regrowing, we need to recompute the solution path σ from p_current to goal using the updated tree τ
            # - This will give us a new path that avoids the newly detected obstacle
            current_index = len(sigma_main) - 1
            
            # Step 3.3.7: Resume Movement
            # - Once we have a new path σ, we can resume movement along this path toward
            continue
            
        # Step 3.4: Move to Next Node
        # - Update p_current to the next node in the current path σ
        if viz_callback is not None: # and iterations % viz_interval == 0:
            viz_callback({"planner": "RRT*", "tree": tree}) #, "iteration": iterations, "phase": "planning"})
        
    # Step 4: End when goal reached
    # - Once p_current is within the goal threshold, we can terminate and return the final path taken to reach the goal
            
    return executed_path # placeholder
    