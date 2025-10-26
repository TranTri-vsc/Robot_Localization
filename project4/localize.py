import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from example_interfaces.msg import Float32, UInt8
from nav_msgs.msg import OccupancyGrid
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D



class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter')

        # Parameters
        self.num_particles = 1500
        self.map_metadata = None  # Placeholder for map metadata
        self.map_data = None      # Placeholder for map data
        self.map_received = False  # Flag to check if the map is received
        self.particles = None
        self.weights = None
        self.latest_compass_theta = None  # Store the latest compass reading

        # Initialization flag
        self.particles_initialized = False  # Tracks if particles have been initialized

        # For motion model
        self.last_twist_time = None  # Timestamp of the last Twist message

        # For resampling
        self.floor_sensor_count = 0  # Count of messages received from /floor_sensori

        # Counter to track invalid particles
        self.invalid_particle_count = np.zeros(self.num_particles, dtype=int)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/world_map', self.map_callback, 10)
        self.create_subscription(Float32, '/compass', self.update_compass, 10)
        self.create_subscription(Twist, '/cmd_vel', self.update_motion, 10)
        self.create_subscription(UInt8, '/floor_sensor', self.update_sensor, 10)

        # Publisher
        self.particle_pub = self.create_publisher(MarkerArray, '/particles', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.pose2d_pub = self.create_publisher(Pose2D, '/estimated_pose', 10)

        # Path trace Marker
        self.path = []  # List to store the history of poses
        self.path_pub = self.create_publisher(Marker, '/pose_path', 10)  # Publisher for the path

        # Timer for particle initialization
        self.init_timer = self.create_timer(0.1, self.try_initialize_particles)

        # Timer for visualizations
        self.timer = self.create_timer(0.1, self.publish_visualizations)  # Run at 10 Hz

        # Timer for Pose2D publishing
        self.pose2d_timer = self.create_timer(2, self.publish_pose2d)  # Run every 2 seconds
        

    # Get map from /world_map topic
    def map_callback(self, msg):
        if not self.map_received:
            self.get_logger().info("Received map data")
            self.map_metadata = msg.info  # Store metadata
            self.map_data = msg.data      # Store the map data
            self.map_received = True

    # Try init particles until first compass value is passed
    def try_initialize_particles(self):
        if self.map_received and not self.particles_initialized:
            if self.latest_compass_theta is not None:
                self.initialize_particles()
                self.particles_initialized = True
                self.get_logger().info("Particles initialized successfully.")
            else:
                self.get_logger().info("Waiting for compass data to initialize particles...")

    # Init the particles
    def initialize_particles(self):
        if not self.map_received:
            self.get_logger().error("Cannot initialize particles: Map not received")
            return

        if self.latest_compass_theta is None:
            self.get_logger().error("Cannot initialize particles: Compass data not received")
            return

        width = self.map_metadata.width
        height = self.map_metadata.height
        resolution = self.map_metadata.resolution
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y

        self.particles = np.zeros((self.num_particles, 3))  # x, y, theta
        valid_particles = 0

        while valid_particles < self.num_particles:
            # Randomly sample particle positions
            x = np.random.uniform(origin_x, origin_x + width * resolution)
            y = np.random.uniform(origin_y, origin_y + height * resolution)
            theta = self.latest_compass_theta  # Use the compass reading for orientation

            # Convert to map grid coordinates
            map_x = int((x - origin_x) / resolution)
            map_y = int((y - origin_y) / resolution)

            # Check if the sampled position is valid
            if 0 <= map_x < width and 0 <= map_y < height:
                tile_index = map_y * width + map_x
                if self.map_data[tile_index] in [100, 0]:  # Valid tiles: dark (100) or light (0)
                    self.particles[valid_particles] = [x, y, theta]
                    valid_particles += 1

        # Initialize weights
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.get_logger().info("Initialized particles based on map dimensions and compass reading")

    # Update the motion model
    def update_motion(self, msg):
        if not self.map_received or self.particles is None:
            return

        # Compute time delta
        now = self.get_clock().now()
        if self.last_twist_time is None:
            dt = 0.1  # Default timestep if no previous timestamp is available
        else:
            dt = (now - self.last_twist_time).nanoseconds / 1e9 -0.001 # Convert nanoseconds to seconds

        self.last_twist_time = now  # Update the last received timestamp

        # Predict particles' motion using cmd_vel
        linear = msg.linear.x
        angular = msg.angular.z
        noise = np.random.normal(0, 0.05, self.num_particles)

        self.particles[:, 2] += angular * dt + noise
        self.particles[:, 0] += linear * dt * np.cos(self.particles[:, 2])
        self.particles[:, 1] += linear * dt * np.sin(self.particles[:, 2])

        # Constrain particles to valid map boundaries
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        width = self.map_metadata.width
        height = self.map_metadata.height

        for i in range(self.num_particles):
            map_x = int((self.particles[i, 0] - origin_x) / resolution)
            map_y = int((self.particles[i, 1] - origin_y) / resolution)

            map_float_x = (self.particles[i, 0] - origin_x) / resolution
            map_float_y = (self.particles[i, 1] - origin_y) / resolution

            # Penalize particles that are near of out of bounds
            if not (0 <= map_float_x < (width - 0.2) and 0 <= map_float_y < (height - 0.2)):
                self.weights[i] *= 0.001  # Apply heavy penalty
            else:
                tile_index = map_y * width + map_x
                if self.map_data[tile_index] not in [100, 0]:  # Invalid tiles
                    self.weights[i] *= 0.001  # Apply heavy penalty

    # Uniformly distribute the particles passed to here
    def reinitialize_particle(self, index):
        width = self.map_metadata.width
        height = self.map_metadata.height
        resolution = self.map_metadata.resolution
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y

        while True:
            # Randomly sample a new position
            x = np.random.uniform(origin_x, origin_x + width * resolution)
            y = np.random.uniform(origin_y, origin_y + height * resolution)
            theta = self.latest_compass_theta if self.latest_compass_theta is not None else np.random.uniform(-np.pi, np.pi)

            # Convert to map grid coordinates
            map_x = int((x - origin_x) / resolution)
            map_y = int((y - origin_y) / resolution)

            # Check if the new position is valid
            if 0 <= map_x < width and 0 <= map_y < height:
                tile_index = map_y * width + map_x
                if self.map_data[tile_index] in [100, 0]:  # Valid tiles: dark (100) or light (0)
                    self.particles[index] = [x, y, theta]
                    break


    # Update the weights using the sensor model (/floor_sensor)
    def update_sensor(self, msg):
        if not self.map_received or self.particles is None:
            return

        observed_color = msg.data
        expected_colors = self.get_expected_colors()

        for i in range(self.num_particles):
            if expected_colors[i] == 'light':
                mu, sigma = 116.5, 9.11
            elif expected_colors[i] == 'dark':
                mu, sigma = 136.14, 8.82
            else:
                self.weights[i] *= 0.001  # Penalize unknown tiles
                continue

            # Compute the weight using Gaussian likelihood
            self.weights[i] *= np.exp(-0.5 * ((observed_color - mu) ** 2) / (sigma ** 2))

        # Handle all-zero weights
        if np.sum(self.weights) == 0:
            self.get_logger().warning("All weights are zero (update_sensor). Resetting to uniform distribution.")
            self.weights = np.ones(self.num_particles) / self.num_particles
        else:
            # Normalize weights
            self.weights /= np.sum(self.weights)

        # Increment the floor sensor count
        self.floor_sensor_count += 1

        # Perform resampling every 5 messages
        if self.floor_sensor_count >= 5:
            self.resample_particles()
            self.floor_sensor_count = 0  # Reset count

    # Resampling step
    def resample_particles(self):
        if self.weights is None or self.particles is None:
            return

        # Ensure weights are valid
        if np.sum(self.weights) == 0:
            self.get_logger().warning("All weights are zero during resampling. Resetting particles.")
            self.weights = np.ones(self.num_particles) / self.num_particles
            return

        # Low-variance resampling
        new_particles = np.zeros_like(self.particles)
        new_weights = np.ones(self.num_particles) / self.num_particles

        index = int(np.random.uniform(0, self.num_particles))
        beta = 0.0
        mw = np.max(self.weights)

        for i in range(self.num_particles):
            beta += np.random.uniform(0, 2.0 * mw)
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % self.num_particles
            new_particles[i] = self.particles[index]

        self.particles = new_particles
        self.weights = new_weights

        # Check for particles that remain invalid after resampling
        self.reinitialize_invalid_particles()

    # Reinitialize particles incase particle falls outside the boundary for too long
    def reinitialize_invalid_particles(self):
        """Reinitialize particles that remain invalid for more than 5 resampling steps."""
        if not self.map_received:
            return

        width = self.map_metadata.width
        height = self.map_metadata.height
        resolution = self.map_metadata.resolution
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y

        for i in range(self.num_particles):
            map_x = int((self.particles[i, 0] - origin_x) / resolution)
            map_y = int((self.particles[i, 1] - origin_y) / resolution)

            # Check if the particle is outside the map or in an invalid tile
            if not (0 <= map_x < width and 0 <= map_y < height):
                self.invalid_particle_count[i] += 1
            else:
                tile_index = map_y * width + map_x
                if self.map_data[tile_index] not in [100, 0]:  # Invalid tiles
                    self.invalid_particle_count[i] += 1
                else:
                    self.invalid_particle_count[i] = 0  # Reset counter if valid

            # Reinitialize particle if it has been invalid for too long
            if self.invalid_particle_count[i] > 3:
                self.invalid_particle_count[i] = 0  # Reset counter
                self.reinitialize_particle(i)

    # Use compass to set the direction of the robot (with some variance)
    def update_compass(self, msg):
        # Store the latest compass reading
        self.latest_compass_theta = msg.data

        # Don't proceed if the map or particles are not initialized
        if not self.map_received or self.particles is None:
            return

        # Introduce a small variance to the compass reading for each particle
        compass_variance = 0.015  # Adjust the variance as needed
        noisy_compass_readings = np.random.normal(self.latest_compass_theta, np.sqrt(compass_variance), self.num_particles)

        # Update particle orientations with the noisy compass readings
        self.particles[:, 2] = noisy_compass_readings

        # Normalize weights to ensure they remain valid
        self.weights = np.ones(self.num_particles) / self.num_particles


    # Get color of the tile where each particle is on
    def get_expected_colors(self):
        expected_colors = []
        for particle in self.particles:
            x, y = particle[0], particle[1]
            # Convert particle position to map grid coordinates
            map_x = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
            map_y = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)

            # Ensure the particle is within map bounds
            if 0 <= map_x < self.map_metadata.width and 0 <= map_y < self.map_metadata.height:
                tile_index = map_y * self.map_metadata.width + map_x
                tile_value = self.map_data[tile_index]  # Use the stored map data
                expected_colors.append('dark' if tile_value == 100 else 'light')
            else:
                expected_colors.append('unknown')  # Out-of-bounds particles
        return expected_colors

    # Publish pose
    def publish_pose(self):
        if not self.map_received or self.particles is None:
            return

        # Compute estimated pose
        x = np.sum(self.particles[:, 0] * self.weights)
        y = np.sum(self.particles[:, 1] * self.weights)
        theta = np.sum(self.particles[:, 2] * self.weights)

        # Publish estimated pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = np.sin(theta / 2)
        pose.pose.orientation.w = np.cos(theta / 2)
        self.pose_pub.publish(pose)

        # Update the path history
        self.path.append((x, y))
        if len(self.path) > 500:  # Limit the path history to avoid memory issues
            self.path.pop(0)

        # Publish the path visualization
        self.publish_path()

    # Publish path for tracing the possible robot pose
    def publish_path(self):
        if not self.path:
            return

        # Create a Marker for the path
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        # Set the path properties
        path_marker.scale.x = 0.05  # Line width
        path_marker.color.a = 1.0  # Alpha (transparency)
        path_marker.color.r = 0.0  # Red
        path_marker.color.g = 0.0  # Green
        path_marker.color.b = 1.0  # Blue (path color)

        # Add points to the path
        for x, y in self.path:
            point = Marker().pose.position.__class__()  # Create a geometry_msgs/Point
            point.x = x
            point.y = y
            point.z = 0.0
            path_marker.points.append(point)

        # Publish the path
        self.path_pub.publish(path_marker)

    # Call vizualization
    def publish_visualizations(self):
        self.publish_pose()
        self.publish_particles()

    # Publish pose2d for compliance
    def publish_pose2d(self):
        if not self.map_received or self.particles is None:
            return

        # Compute estimated pose
        x = np.sum(self.particles[:, 0] * self.weights)
        y = np.sum(self.particles[:, 1] * self.weights)
        theta = np.sum(self.particles[:, 2] * self.weights)

        # Create and publish the Pose2D message
        pose2d = Pose2D()
        pose2d.x = x
        pose2d.y = y
        pose2d.theta = theta
        self.pose2d_pub.publish(pose2d)

    # Publish particles for tracking
    def publish_particles(self):
        if self.particles is None or self.weights is None:
            return

        marker_array = MarkerArray()

        for i, particle in enumerate(self.particles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set particle position
            marker.pose.position.x = particle[0]
            marker.pose.position.y = particle[1]
            marker.pose.position.z = 0.0  # Keep markers on the map plane

            # Set marker orientation (no orientation for spheres)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set marker scale (size of each particle marker)
            marker.scale.x = 0.1  # Width
            marker.scale.y = 0.1  # Height
            marker.scale.z = 0.1  # Depth

            # Set marker color based on particle weight
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 1.0 - self.weights[i]  # Red decreases with weight
            marker.color.g = self.weights[i]        # Green increases with weight
            marker.color.b = 0.0                    # No blue component

            # Add the marker to the MarkerArray
            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.particle_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
