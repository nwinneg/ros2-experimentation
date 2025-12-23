import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path                           # The path message
from geometry_msgs.msg import PoseStamped               # Each waypoint in the path
from geometry_msgs.msg import Pose, Point, Quaternion   # Convenience
from std_msgs.msg import Header                         # For setting headers
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry, Path  # for /map
import math        # for distance, angles
import numpy as np # if doing path computations like A*, grids, etc.
from dataclasses import dataclass

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

        # Publisher to publish computed path
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

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
        pass

    def astar(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        """
        start, goal: tuple[int, int] = (row, col)
        returns: list of tuples (row, col) from start to goal or None
        """
        pass

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
        self.get_logger().info(f"Planning route from {current} to {goal}")
    
        # Use function to do A*

        # Convert from map to world coords

        # Publish path

        self.get_logger().info("Got into route planning block and past pose tranformation.")



def main(args=None):
    rclpy.init(args=args)
    planner = SimplePlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

        