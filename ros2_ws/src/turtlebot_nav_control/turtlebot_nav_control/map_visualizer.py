import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')

        # Subscribe to map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.get_logger().info("Waiting for map data...")

    def map_callback(self, msg):
        self.get_logger().info(f"Received map data: {msg.info.width}x{msg.info.height}")

        # Extract map data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Reshape 1D incoming data into 2D grid
        data = np.array(msg.data).reshape((height, width))

        # Find occupied cells
        occupied_y_indices, occupied_x_indices = np.where(data > 100)

        # Convert Grid Indices to World Coordinates (Meters) --> World = (Index * Resolution) + Origin
        world_x = (occupied_x_indices * resolution) + origin_x
        world_y = (occupied_y_indices * resolution) + origin_y

        # Set up plotting figure
        self.get_logger().info("Plotting map...")
        plt.figure(figsize=(10,10))

        # Plot occupied points as black dots
        plt.scatter(world_x, world_y, c='black', s=1, marker='s')
        
        # Set plot limits to match the map size reasonably
        plt.axis('equal')
        plt.title("Occupancy Grid Visualized in World Coordinates")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.grid(True)
        plt.show()

        # Shutdown after one plot so we don't freeze with infinite popups
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()
    rclpy.spin(map_visualizer)