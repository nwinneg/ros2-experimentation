import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf2_ros
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose
from rclpy.time import Time

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Create publisher to publish velocity commands and subscription to read odometry data
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom',self.odom_callback,10)

        # Initialize a timer to control update rate of velocity commands
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Get waypoints from subscription to goal pose in rviz
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Initialize transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define waypoints (map frame)
        self.goal_x = None
        self.goal_y = None
        self.reached_target = None
        self.goal_frame = None

        # Store current pose and header (from odom callback)
        self.current_pose = None
        self.odom_header = None

    def goal_callback(self,msg):

        # Set target pose based on msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_frame = msg.header.frame_id
        self.get_logger().info(
            f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )
        self.reached_target = False

    def odom_callback(self,msg):
        
        # Set pose and header
        self.current_pose = msg.pose.pose
        self.odom_header = msg.header

    def control_loop(self):

        # Now runs on a timer
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):

        # Check for target waypoint
        if (self.goal_x is None) or (self.goal_y is None):
            cmd = TwistStamped() # Stay stopped
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            self.get_logger().info("No target waypoint set")
            return

        # Check for current pose
        if (self.current_pose is None) or (self.odom_header is None):
            cmd = TwistStamped() # Stay stopped
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            self.get_logger().info("No current pose available")
            return

        # If we have a current pose, tranform it to the map frame and set current_x, current_y, current_yaw
        target_frame = self.goal_frame
        source_frame = 'odom'

        try: 
            # Get transform from odom frame to map frame
            transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    # rclpy.time.Time(),
                    Time.from_msg(self.odom_header.stamp),
                    Duration(seconds=2.0) 
                )

            # Create PostStamped pose from current pose
            current_pose_stamped = PoseStamped()
            current_pose_stamped.header = self.odom_header # Frame ID and Stamp
            current_pose_stamped.pose = self.current_pose

            # Transform current pose into map frame  
            transformed_pose = do_transform_pose(current_pose_stamped.pose, transform)
            
            # Set current position from transformed pose
            current_x = transformed_pose.position.x
            current_y = transformed_pose.position.y

            # Get current heading
            orientation_q = transformed_pose.orientation # Orientation passed in the form of a quaternion
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _, _, current_yaw = euler_from_quaternion(orientation_list)

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {source_frame} to {target_frame}: {ex}"
            )
            cmd = TwistStamped() # Stay stopped
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            return

        # Get current position
        dx = self.goal_x - current_x
        dy = self.goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate desired heading (yaw angle) and heading error
        desired_yaw = math.atan2(dy,dx)
        yaw_error = desired_yaw - current_yaw
        yaw_error = (yaw_error + np.pi) % (2*np.pi) - np.pi  # Normalize to [-pi, pi]

        # Create a command structure
        cmd = TwistStamped()

        # Do simple control logic
        if distance > 0.1: # Not at waypoint yet

            # Rotate to face waypoint
            desired_angular = np.clip(1.5 * yaw_error,-2.5,2.5)   # Proportional yaw controller (clipped)

            # Move forward once roughly facing target
            if abs(yaw_error) < (30 * (np.pi/180)):
                desired_linear = min(0.7, 0.7 * distance)  # Proportional speed with a ceiling
            else:
                desired_linear = 0.0  # Stop and turn first

        else: # At waypoint
            desired_linear = 0.0
            desired_angular = 0.0
            if not self.reached_target:
                self.get_logger().info(f"Waypoint ({self.goal_x:.2f}, {self.goal_y:.2f}) reached")
            self.reached_target = True

        # Apply desired commands and publish
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = desired_linear
        cmd.twist.angular.z = desired_angular
        self.cmd_pub.publish(cmd)

# Main function for launching in simulation
def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator() # Instance of dumb navigator

    try:
        rclpy.spin(navigator) # Spin the node - infinite loop that keeps the process running
    except KeyboardInterrupt:
        pass

    navigator.destroy_node() # Cleanup resources when spin closes (CTRL+C)
    rclpy.shutdown() # Shutdown when done

