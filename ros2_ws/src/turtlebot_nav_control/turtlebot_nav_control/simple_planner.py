import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path                           # The path message
from geometry_msgs.msg import PoseStamped               # Each waypoint in the path
from geometry_msgs.msg import Pose, Point, Quaternion   # Convenience
from std_msgs.msg import Header                         # For setting headers
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped               # for /goal_pose
from nav_msgs.msg import OccupancyGrid                  # for /map
import math        # for distance, angles
import numpy as np # if doing path computations like A*, grids, etc.

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        