import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped               # Each waypoint in the path
from tf_transformations import quaternion_from_euler
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # for /map

# For setting QoS profile of /planned_path topic
from rclpy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

# Mostly used in A* algorithm
import math
from dataclasses import dataclass
import heapq

# Create data class to store map information
@dataclass(frozen=True)
class MapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')

        # Physical paramters
        self.radius = 0.22      # Radius of the robot for inflation purposes

        # Set QoS profile for path publisher
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,     
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Latch last message for late subscribers
        )

        # Publisher to publish computed path
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            path_qos
        )

        # Subscriber to receive goal pose from rviz
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Subscribe to odometry messages for localization
        self.pose_sub = self.create_subscription(Odometry,'/diff_drive_controller/odom',self.odom_callback,10)

        # Subscribe to the map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Initialize transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define goal waypoint (map frame)
        self.goal_x = None
        self.goal_y = None
        self.reached_target = None
        self.goal_frame = None

        # Store current pose and header (from odom callback)
        self.current_pose = None
        self.odom_header = None

        # Store the current map as received by map subscription
        self.map = None
        self.map_info: MapInfo | None = None

    def goal_callback(self, msg): 
        # Receive the goal pose, store it in a member variable, and plan a new path when received

        # Receive and store the goal pose
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_frame = msg.header.frame_id
        self.get_logger().info(
            f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

        # Plan a route
        self.plan_route()

    def odom_callback(self, msg):
        # Receive odom messages and store the current pose for path start point
        self.current_pose = msg.pose.pose
        self.odom_header = msg.header

    def map_callback(self,msg):

        # Extract map info
        self.map_info = MapInfo(
            width = msg.info.width,
            height = msg.info.height,
            resolution = msg.info.resolution,
            origin_x = msg.info.origin.position.x,
            origin_y = msg.info.origin.position.y
        )

        # Keep as 1D array for speed, store incoming map
        self.map = msg.data # Note: Accessible by ind = row * width + col

    def transform_odom_to_map(self):
        # Set source and target frames
        target_frame = self.goal_frame
        source_frame = self.odom_header.frame_id

        # Check that there is a transform
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(self.odom_header.stamp),
                Duration(seconds=2.0)
            )

            # Apply the transform and return current pose
            current_pose_stamped = PoseStamped()
            current_pose_stamped.header = self.odom_header # Frame ID and Stamp
            current_pose_stamped.pose = self.current_pose
            
            transformed_pose = do_transform_pose(current_pose_stamped.pose, transform)

            return transformed_pose

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {source_frame} to {target_frame}: {ex}"
            )
            return None

    def map_to_grid(self, x: float, y: float) -> tuple[int, int]:
        # Retrieve grid coordinates from map coordinates
        mi = self.map_info

        # Convert from map coordinates to grid coordinates (row: y, col: x)
        col = math.floor((x - mi.origin_x) / mi.resolution)
        row = math.floor((y - mi.origin_y) / mi.resolution)

        return row, col

    def grid_to_map(self, row: int, col: int) -> tuple[float, float]:
        # Make sure we have map info and get it
        if self.map_info is None:
            raise RuntimeError("Map not available")

        mi = self.map_info

        # Convert grid indices to world coordinates (cell center)
        x = mi.origin_x + (col + 0.5) * mi.resolution
        y = mi.origin_y + (row + 0.5) * mi.resolution

        return x, y

    def is_free(self, row: int, col: int) -> bool:
        # Check that a cell is free, consider robot radius for inflation
        # -1: unknown, 0: free, 1-100: occupied with some probability
        
        # Get map info and map data
        mi = self.map_info
        map = self.map

        # Convert robot radius into grid cells
        inflation_cells = math.ceil(self.radius / mi.resolution)

        # Check if the cell is free
        for i in range(-inflation_cells, inflation_cells + 1):
            for j in range(-inflation_cells, inflation_cells + 1):
                # Get cell index
                r = row + i
                c = col + j

                # Check if cell is within bounds
                if (r < 0) or (c < 0) or (r >= mi.height) or (c >= mi.width):
                    return False

                # Check if cell is free
                if map[r * mi.width + c] > 0:
                    return False
        
        return True

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        """
        start, goal: tuple[int, int] = (row, col)
        returns: list of tuples (row, col) from start to goal or None
        """
        
        mi = self.map_info

        # Initialize priority queue to store path (fcost, (row, col))
        open_set = [] # Priority queue
        closed_set = set() # Set of visited nodes
        heapq.heappush(open_set, (0.0, start))

        # Initialize dictionaries to store cost and path 
        came_from = {} # Parent of each node
        g_cost = {start : 0.0} # Cost to reach each node

        # Helper function to get the heuristic cost (h-cost) where b is the goal
        def heuristic(a, b):
            # self.get_logger().info(f"Heuristic cost: {math.hypot(a[0] - b[0], a[1] - b[1])}")
            return math.hypot(a[0] - b[0], a[1] - b[1])

        # Define connected neighbors for a grid point
        neighbors = [
            (-1, 0), # Up
            (1, 0),  # Down
            (0, -1), # Left
            (0, 1),   # Right
            (-1, -1),# Up left
            (-1, 1), # Up right
            (1, -1),  # Down left
            (1, 1)   # Down right
        ]

        # Loop until goal reached or open_set is empty
        while open_set:
            # Get the node with the lowest f_cost
            _, current = heapq.heappop(open_set)

            # Check if current node is in closed set
            if current in closed_set:
                continue
            
            # Add current node to closed set
            closed_set.add(current)

            # If goal reached, reconstruct and return path
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            # Get current g_cost
            cur_g = g_cost[current]

            # Loop over neighbors
            for dr, dc in neighbors:
                # Get neighbor coordinates
                nr = current[0] + dr
                nc = current[1] + dc

                # Check that neighbor is in bounds
                if not ((0 <= nr < mi.height) and (0 <= nc < mi.width)):
                    continue
                
                # Check that neighbor is free
                if not self.is_free(nr, nc):
                    continue

                # Calculate step cost of travel to neighbor and tentative g cost if chosen
                step_cost = math.hypot(dr, dc) # 1 or sqrt(2) for diagonal
                tentative_g_cost = cur_g + step_cost
                neighbor = (nr, nc)

                # Check if neighbor is already in the open set, if not, add to open_set
                if tentative_g_cost < g_cost.get(neighbor, float('inf')):
                    # Update path and g_cost
                    came_from[neighbor] = current
                    g_cost[neighbor] = tentative_g_cost
                    
                    # Calculate and push f_cost to open_set
                    f_cost = tentative_g_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_cost, neighbor))
        
        # If goal not reached, return None
        return None

    def _reconstruct_path(self, came_from, current):
        # Reconstruct path from start to goal
        path = [current] # Start at the goal

        while current in came_from: # While we have a parent
            # Go backwards through the path and append
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def plan_route(self):
        # Plan route from current position to goal position and publish
        
        # Base cases
        if (self.goal_x is None) or (self.goal_y is None):
            self.get_logger().info("Planner did not receive goal pose.")
            return
        
        if (self.current_pose is None):
            self.get_logger().info("Waiting for odometry data.")
            return

        if (self.map is None):
            self.get_logger().info("Waiting for map data.")
            return

        # Get pose in global frame
        current_pose = self.transform_odom_to_map()
        if current_pose is None:
            self.get_logger().info("Unable to transform current pose")
            return
        
        # Get coordinates and start planning a route
        current = self.map_to_grid(current_pose.position.x, current_pose.position.y)
        goal = self.map_to_grid(self.goal_x, self.goal_y)

        # Use function to do A*
        path = self.astar(current, goal)
        if path is None:
            self.get_logger().info("A* failed to find a path.")
            return

        # Convert from map to world coords
        waypoints = []
        for row, col in path:
            position = self.grid_to_map(row, col)
            waypoints.append(position)

        # Create path msg
        path_msg = Path()
        path_msg.header.frame_id = self.goal_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Loop through waypoints and add to path
        for ind in range(len(waypoints)):

            # Get waypoint coordinates
            x, y = waypoints[ind]

            # Calculate desired yaw if not last waypoint
            if ind < len(waypoints) - 1:
                x_next, y_next = waypoints[ind + 1]
                yaw = math.atan2(y_next - y, x_next - x)
            
            # Create a pose msg
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.goal_frame
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            # Set cart pose components
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0

            # Calculate quaternion from yaw
            q = quaternion_from_euler(0.0, 0.0, yaw)

            # Set rotation components
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            # Append to path message
            path_msg.poses.append(pose_msg)
        
        # Publish path
        self.path_pub.publish(path_msg)

        self.get_logger().info(f"Path published from {current} to {goal}")

def main(args=None):
    rclpy.init(args=args)
    planner = SimplePlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

        