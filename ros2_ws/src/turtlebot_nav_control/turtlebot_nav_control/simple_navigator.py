import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Create publisher to publish velocity commands and subscription to read odometry data
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom',self.odom_callback,10)

        # Initialize a timer to control update rate of velocity commands
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Get waypoints from subscription to goal pose in rviz
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Define waypoints
        self.target_x = None
        self.target_y = None
        self.reached_target = None

        # Initialize current state member variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def goal_callback(self,msg):

        # Set target pose based on msg
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.get_logger().info(
            f"New goal received: ({self.target_x:.2f}, {self.target_y:.2f})"
        )
        self.reached_target = False

    def odom_callback(self,msg):
        
        # Get current position from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y 

        # Get current heading
        orientation_q = msg.pose.pose.orientation # Orientation passed in the form of a quaternion
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)


    def control_loop(self):

        # Now runs on a timer
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):

        # Check for target waypoint
        if (self.target_x is None) or (self.target_y is None):
            cmd = TwistStamped() # Stay stopped
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(cmd)
            return

        # Get current position
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate desired heading (yaw angle) and heading error
        desired_yaw = math.atan2(dy,dx)
        yaw_error = desired_yaw - self.current_yaw
        yaw_error = (yaw_error + np.pi) % (2*np.pi) - np.pi  # Normalize to [-pi, pi]

        # Create a command structure
        cmd = TwistStamped()

        # Do simple control logic
        if distance > 0.1: # Not at waypoint yet

            # Rotate to face waypoint
            desired_angular = 1.5 * yaw_error   # Proportional yaw controller (no ceiling)

            # Move forward once roughly facing target
            if abs(yaw_error) < (30 * (np.pi/180)):
                desired_linear = min(0.7, 0.7 * distance)  # Proportional speed with a ceiling
            else:
                desired_linear = 0.0  # Stop and turn first

        else: # At waypoint
            desired_linear = 0.0
            desired_angular = 0.0
            if not self.reached_target:
                self.get_logger().info(f"Waypoint ({self.target_x:.2f}, {self.target_y:.2f}) reached")
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

