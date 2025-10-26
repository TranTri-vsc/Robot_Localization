import rclpy
from rclpy.node import Node
from example_interfaces.msg import UInt8
import matplotlib.pyplot as plt
from datetime import datetime

class FloorSensorPlotter(Node):
    def __init__(self):
        super().__init__('floor_sensor_plotter')

        # Initialize storage for sensor data and time
        self.sensor_data = []
        self.time_data = []
        self.start_time = None

        # Create a subscriber to /floor_sensor
        self.subscription = self.create_subscription(
            UInt8,
            '/floor_sensor',
            self.sensor_callback,
            10
        )

        # Notify the user
        self.get_logger().info("Subscribed to /floor_sensor")

    def sensor_callback(self, msg):
        current_time = datetime.now()

        if self.start_time is None:
            # Set the start time when the first message is received
            self.start_time = current_time

        # Calculate the elapsed time in seconds
        elapsed_time = (current_time - self.start_time).total_seconds()

        # Store the sensor data and time
        self.sensor_data.append(msg.data)
        self.time_data.append(elapsed_time)

        self.get_logger().info(f"Received data: {msg.data} at time: {elapsed_time:.2f}s")

    def save_data(self, filename='floor_sensor_data.txt'):
        if not self.sensor_data:
            print("No data received to save.")
            return

        with open(filename, 'w') as f:
            for time, value in zip(self.time_data, self.sensor_data):
                f.write(f"{time},{value}\n")
        print(f"Data saved to {filename}")

    def plot_data(self):
        # Plot the data once the node is shut down
        if not self.sensor_data or not self.time_data:
            print("No data received to plot.")
            return

        plt.figure(figsize=(10, 5))
        plt.plot(self.time_data, self.sensor_data, marker='o', linestyle='-')
        plt.title('Floor Sensor Data Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Floor Sensor Reading')
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = FloorSensorPlotter()

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Save data and plot the graph before shutting down
        node.save_data('floor_sensor_data.txt')
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
