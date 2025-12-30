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
        self.odom_header = None
        self.path = None
        self.reached_goal = False
        self.goal_distance_thresh = 0.2 # Meters
        
        # TODO: Control parameters
        self.look_ahead_index = 2 # Index of look-ahead point in path
        self.k_x = 1.0
        self.k_y = 1.0
        self.k_theta = 1.0

        # TODO: Other member variables as needed (Maybe errors, previous states for smoothing, etc)
        self.ey_history = [] # For smoothing (stanley style controller)
        self.velocity_limits = (0.0, 0.5) # Min and max linear velocity (m/s)
        self.angular_velocity_limits = (-1.0, 1.0) # Min and max angular velocity (rad/s)

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

    def world_to_odom(self,pose_stamped,target_frame, header=None):
        # Function to transform a pose from world (map) frame to odom frame
        if header is None:
            source_frame = pose_stamped.header.frame_id
            msg_time = Time.from_msg(pose_stamped.header.stamp)
            pose = pose_stamped.pose
        else:
            source_frame = header.frame_id
            msg_time = Time.from_msg(header.stamp)
            pose = pose_stamped

        try: 
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                msg_time,
                rclpy.duration.Duration(seconds=2.0)
            )

            # Transform pose
            transformed_pose = do_transform_pose(pose, transform)

            return transformed_pose
        
        except TransformException as ex:
            self.get_logger().info(
                f"Transform from {source_frame} to {target_frame} failed: {ex}"
            )
            return None
        
    def check_goal_reached(self):
        # Function to check if robot has reached the goal
        goal_pose = self.path.poses[-1]
        if self.current_pose is None:
            return None

        # Calculate distance to goal
        distance_to_goal = math.sqrt(
            (self.current_pose.position.x - goal_pose.pose.position.x) ** 2 +
            (self.current_pose.position.y - goal_pose.pose.position.y) ** 2
        )

        # Determine if within threshold
        if distance_to_goal < self.goal_distance_thresh:
            if not self.reached_goal:
                self.get_logger().info("Goal Reached!")
                self.reached_goal = True
            return True
        else:
            self.reached_goal = False
            return False

    def find_nearest_path_point(self):
        # Function to find nearest point on path to current robot pose
        # TODO:
        #   self.path is a Path type message, which is a python list of PoseStamped msgs
        #   These need to be compared to the robot's current position in the world frame
        #   Return the index of the point nearest the robot in the path
        
        # Ensure we have a path and a odometry data
        if self.path is None or len(self.path.poses) == 0 or self.current_pose is None:
            return None
        
        # Transform current pose to world frame
        target_frame = self.path.poses[0].header.frame_id
        header = self.odom_header        
        current_pose_world = self.world_to_odom(self.current_pose,target_frame,header)
        if current_pose_world is None:
            return None
        
        # Get current x,y in world frame
        current_x = current_pose_world.position.x
        current_y = current_pose_world.position.y

        # Find nearest path point
        nearest_index = 0
        min_distance = float('inf')
        for i, path_pose_stamped in enumerate(self.path.poses):
            path_x = path_pose_stamped.pose.position.x
            path_y = path_pose_stamped.pose.position.y
            distance = math.sqrt((current_x - path_x) ** 2 + (current_y - path_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
        
        return nearest_index

    def compute_tracking_errors(self, nearest_index):
        # Function to compute lateral and heading errors given a nearest and look-ahead index
        # Lookahead index is in self.look_ahead_index ahead
        # TODO:
        #   First transform the robot pose and path point to the same frame (odom)
        #       They should both be in the world frame to start, will be called out in header
        #   Then compute and return ex, ey, etheta
        
        # Ensure we have a path and a odometry data
        if self.path is None or len(self.path.poses) == 0 or self.current_pose is None:
            return None, None, None, None

        # Get look-ahead index and pose
        look_ahead_index = min(nearest_index + self.look_ahead_index, len(self.path.poses) - 1)
        look_ahead_pose_stamped = self.path.poses[look_ahead_index]

        # Transform target pose to odom frame (robot frame)
        target_frame = self.odom_header.frame_id
        header = look_ahead_pose_stamped.header
        look_ahead_pose_odom = self.world_to_odom(look_ahead_pose_stamped,target_frame)
        if look_ahead_pose_odom is None:
            return None, None, None
        
        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, current_yaw = euler_from_quaternion(orientation_list)

        # Get target position
        target_x = look_ahead_pose_odom.position.x
        target_y = look_ahead_pose_odom.position.y
        target_orientation_q = look_ahead_pose_odom.orientation
        target_orientation_list = [target_orientation_q.x, target_orientation_q.y, target_orientation_q.z, target_orientation_q.w]
        _, _, target_yaw = euler_from_quaternion(target_orientation_list)

        # Compute errors
        dx = target_x - current_x
        dy = target_y - current_y

        ex = math.cos(current_yaw) * dx + math.sin(current_yaw) * dy
        ey = -math.sin(current_yaw) * dx + math.cos(current_yaw) * dy
        etheta = target_yaw - current_yaw
        etheta = (etheta + math.pi) % (2 * math.pi) - math.pi # Wrap to [-pi, pi]
        
        return ex, ey, etheta
    
    def generate_control_command(self):
        # Main function to generate control commands based on current pose and path
        # TODO:
        #   Find nearest path point index
        #       Current pose will need to be transformed to world frame
        #   Compute tracking errors
        #   Compute control commands (linear and angular velocities)
        #   Publish control commands

        # Helper function to clamp cmd values
        def clamp(value, min_value, max_value):
            return max(min(value, max_value), min_value)
        
        # Check there is a path and get goal if so
        if self.path is None or len(self.path.poses) == 0:
            return
        goal_reached = self.check_goal_reached()

        # If goal reached or no current pose, publish zero velocities
        if (goal_reached == None) or (goal_reached == True):
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'odom'
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return
        
        # Find nearest path point index
        nearest_index = self.find_nearest_path_point()
        if nearest_index is None:
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'odom'
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return

        # Compute tracking errors
        ex, ey, etheta = self.compute_tracking_errors(nearest_index)
        if ex is None:
            cmd_msg = TwistStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'odom'
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return

        # Compute control commands
        v = self.k_x * ex
        omega = self.k_y * ey + self.k_theta * etheta

        # Handle saturation limits
        v = clamp(v, self.velocity_limits[0], self.velocity_limits[1])
        omega = clamp(omega, self.angular_velocity_limits[0], self.angular_velocity_limits[1])

        # Publish control commands
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = self.odom_header.frame_id
        cmd_msg.twist.linear.x = v
        cmd_msg.twist.angular.z = omega
        self.cmd_pub.publish(cmd_msg)
        

def main(args=None):
    rclpy.init(args=args)
    simple_follower = SimpleFollower()
    rclpy.spin(simple_follower)
    simple_follower.destroy_node()
    rclpy.shutdown()