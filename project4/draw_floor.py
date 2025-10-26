import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
import sys

class WorldPublisher(Node):
    def __init__(self, world_file):
        super().__init__('world_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/world_map', 10)
        self.world_file = world_file
        self.grid_data = None  # Placeholder for the map data

        # Process the map and save it
        self.process_world_file()
        
        # Start a timer to publish the map every 0.1 seconds
        self.timer_ = self.create_timer(0.1, self.publish_world)

    def process_world_file(self):
        try:
            with open(self.world_file, 'r') as f:
                world_data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"File {self.world_file} not found!")
            return

        resolution = world_data.get('resolution', 1.0)
        raw_map = world_data.get('map', '').strip().split('\n')

        # Create OccupancyGrid
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = resolution
        grid.info.width = len(raw_map[0])
        grid.info.height = len(raw_map)
        grid.info.origin = Pose()
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0

        # Convert map to grid data
        data = []
        for row in reversed(raw_map):  # Flip rows to match RViz's coordinate system
            for char in row:
                if char == '#':
                    data.append(100)  # Occupied cell
                elif char == '.':
                    data.append(0)    # Free cell
                else:
                    data.append(-1)   # Unknown cell

        grid.data = data
        self.grid_data = grid  # Save the grid for reuse

        self.get_logger().info(f"Processed and pub world map from {self.world_file}.")

        if self.grid_data:
            # Update the header's timestamp
            self.grid_data.header.stamp = self.get_clock().now().to_msg()
            # Publish the saved map data
            self.publisher.publish(self.grid_data)
        else:
            self.get_logger().error("No map data to publish.")

    def publish_world(self):
        if self.grid_data:
            # Update the header's timestamp
            self.grid_data.header.stamp = self.get_clock().now().to_msg()
            # Publish the saved map data
            self.publisher.publish(self.grid_data)
            self.get_logger().info(f"Published the world map from {self.world_file}.")
        else:
            self.get_logger().error("No map data to publish.")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run sim_localize draw_floor.py <path_to_world_file>")
        return

    world_file = sys.argv[1]
    node = WorldPublisher(world_file)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
