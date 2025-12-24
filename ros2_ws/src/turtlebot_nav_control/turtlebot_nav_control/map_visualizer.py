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

        # Notes: 
        #   0 denotes definitely free space
        #   -1 denotes unknown space
        #   100 denotes definitely occupied space
        #   Values in between 1 and 99 inclusive represent probabilities

        # Find occupied cells
        occupied_y_indices, occupied_x_indices = np.where(data > 50) # 1-1

        # Find unknown cells
        unknown_y_indices, unknown_x_indices = np.where(data == -1) # -1 denotes unknown


        # Convert Grid Indices to World Coordinates (Meters) --> World = (Index * Resolution) + Origin
        occupied_world_x = (occupied_x_indices * resolution) + origin_x
        occupied_world_y = (occupied_y_indices * resolution) + origin_y

        # Unknown cells
        unknown_world_x = (unknown_x_indices * resolution) + origin_x
        unknown_world_y = (unknown_y_indices * resolution) + origin_y


        # Set up plotting figure
        self.get_logger().info("Plotting map...")
        plt.figure(figsize=(10,10))

        # Plot occupied points as black dots
        # plt.scatter(world_x, world_y, c='black', s=1, marker='s')
        plt.scatter(unknown_world_x, unknown_world_y, c='lightgray', s=3, marker='s', label='Unknown')
        plt.scatter(occupied_world_x, occupied_world_y, c='black', s=3, marker='s', label='Occupied')
        plt.scatter(origin_x, origin_y, c='green', s=10, marker='o', edgecolors='darkgreen', linewidths=2, label='Origin', zorder=5)

        # Set plot limits to match the map size reasonably
        plt.axis('equal')
        plt.title("Occupancy Grid Visualized in World Coordinates")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.legend()
        plt.grid(True)
        plt.show()

        # Shutdown after one plot so we don't freeze with infinite popups
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()
    rclpy.spin(map_visualizer)