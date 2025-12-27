# Import ROS2 libraries and message types
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path

# Import transform libraries and functions
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

# Import python specific libraries
import math
import numpy as np

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')

        # Create publisher for control cmds w/ standard QoS
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10)

        # Subscribe to odometry msgs for localization
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Subscribe to the path to get trajectory
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        # Initialize timer to control update rate of cmds
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Initialize transform buffer/listener for odom->map transformations
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TODO: Member variables to store pose and path
        self.current_pose = None
        self.path = None
        
        # TODO: Control parameters
        self.look_ahead_index = 5 # Index of look-ahead point in path
        self.k_x = 1.0
        self.k_y = 1.0
        self.k_theta = 1.0

        # TODO: Other member variables as needed (Maybe errors, previous states for smoothing, etc)
        self.ey_history = [] # For smoothing (stanley style controller)

    def odom_callback(self,msg):
        # Callback to receive odom msgs and update member vars
        self.current_pose = msg.pose.pose
        self.odom_header = msg.header

    def path_callback(self,msg):
        # Callback to receive path msgs and update member vars
        self.path = msg
        self.get_logger().info(f"Received new path: \n{msg}")

    def control_loop(self):
        # Function tied to timer to run lateral and speed controllers predictably
        self.generate_control_command()

    def world_to_odom(self,pose_stamped):
        # Function to transform a pose from world (map) frame to odom frame
        pass

    def find_nearest_path_point(self):
        # Function to find nearest point on path to current robot pose
        # TODO:
        #   self.path is a Path type message, which is a python list of PoseStamped msgs
        #   These need to be compared to the robot's current position in the world frame
        #   Return the index of the point nearest the robot in the path
        pass

    def compute_tracking_errors(self, nearest_index):
        # Function to compute lateral and heading errors given a nearest and look-ahead index
        # Lookahead index is in self.look_ahead_index ahead
        # TODO:
        #   First transform the robot pose and path point to the same frame (odom)
        #       They should both be in the world frame to start, will be called out in header
        #   Then compute and return ex, ey, etheta
        pass
    
    def generate_control_command(self):
        # Main function to generate control commands based on current pose and path
        # TODO:
        #   Find nearest path point index
        #       Current pose will need to be transformed to world frame
        #   Compute tracking errors
        #   Compute control commands (linear and angular velocities)
        #   Publish control commands
        pass


def main(args=None):
    rclpy.init(args=args)
    simple_follower = SimpleFollower()
    rclpy.spin(simple_follower)
    simple_follower.destroy_node()
    rclpy.shutdown()