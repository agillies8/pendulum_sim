import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import quaternion  # This library must be installed via `pip install numpy-quaternion`


class PendulumTFNode(Node):
    def __init__(self):
        super().__init__('pendulum_tf_node')

        # Create a TransformListener object
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for the pendulum state
        self.publisher = self.create_publisher(Float32MultiArray, 'pendulum_state', 10)

        # Subscribe to the clock topic
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)

        # Variables to store previous positions and times
        self.prev_cart_x = None
        self.prev_time = None
        self.prev_total_rotation = None


        # List to store cart velocities for plotting
        self.cart_velocities = []
        self.times = []

        # Initialize the plotter
        self.plotter = Plotter(self.times, self.cart_velocities)

        #to track angles wrapping 306 or negative past zero
        self.tracker = AngleTracker()

        self.get_logger().info('Pendulum TF Node has been started.')

    def clock_callback(self, msg):
        # Extract the time from the clock message
        current_time = msg.clock

        try:
            # Lookup transforms
            cart_transform = self.tf_buffer.lookup_transform('world', 'cart', rclpy.time.Time())

            theta_transform: TransformStamped = self.tf_buffer.lookup_transform('rail', 'arm_tip', rclpy.time.Time())

            
            # Extract the z-axis component from the quaternion
            quaternion = [theta_transform.transform.rotation.x, theta_transform.transform.rotation.y, theta_transform.transform.rotation.z, theta_transform.transform.rotation.w]

            world_z_axis = np.array([0, 0, 1])

            arm_z_axis = self.rotate_vector_by_quaternion(world_z_axis, quaternion)

            # Calculate the angle
            angle = self.angle_between_vectors_clockwise(arm_z_axis, world_z_axis)

            
            total_rotation = self.tracker.update_angle(np.degrees(angle))


            # Calculate positions and velocities
            x = cart_transform.transform.translation.x
            cart_x, cart_x_dot = self.calculate_position_and_velocity(
                x, self.prev_cart_x, self.prev_time, current_time)

            total_rotation, total_rotation_dot = self.calculate_position_and_velocity(
                total_rotation, self.prev_total_rotation, self.prev_time, current_time)


            # Update previous positions and times
            self.prev_cart_x = cart_x
            self.prev_total_rotation = total_rotation
            self.prev_time = current_time

            # Update the velocity list for plotting
            self.times.append(current_time.sec + current_time.nanosec * 1e-9)
            self.cart_velocities.append(np.radians(total_rotation_dot))
            # self.plotter.update_plot()


            # Create and publish Float32MultiArray message
            msg = Float32MultiArray()
            msg.data = [cart_x, cart_x_dot, np.radians(total_rotation), np.radians(total_rotation_dot)]
            self.publisher.publish(msg)

            # self.get_logger().info(f'Published pendulum state: {msg.data}')
            
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

    def calculate_position_and_velocity(self, value, prev_x, prev_time, current_time):
        # Extract positions
        x = value

        # Calculate velocity
        if prev_x is not None and prev_time is not None:
            dt = (Time.from_msg(current_time) - Time.from_msg(prev_time)).nanoseconds / 1e9
            x_dot = (x - prev_x) / dt if dt > 0 else 0.0
        else:
            x_dot = 0.0

        return x, x_dot


    def rotate_vector_by_quaternion(self, vector, quaternion):
        # Normalize the quaternion to ensure it's a unit quaternion
        q = np.quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        q_normalized = q.normalized()

        # Convert the vector into a quaternion with zero real part
        v = np.quaternion(0, vector[0], vector[1], vector[2])

        # Perform the quaternion multiplication (rotation)
        v_rot = q_normalized * v * q_normalized.inverse()

        # Return the rotated vector components
        return np.array([v_rot.x, v_rot.y, v_rot.z])

    def angle_between_vectors_clockwise(self, v1, v2, normal_direction=np.array([0, -1, 0])):
        # Calculate the dot product
        dot_product = np.dot(v1, v2)

        # Calculate the norms of each vector
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        # Calculate the cosine of the angle
        cos_theta = dot_product / (norm_v1 * norm_v2)
        cos_theta = max(min(cos_theta, 1), -1)  # Clamp value to the range [-1, 1] for numerical stability

        # Calculate the angle in radians
        angle_rad = np.arccos(cos_theta)

        # Determine the direction using the cross product and the reference normal_direction vector
        cross_product = np.cross(v1, v2)
        direction = np.dot(cross_product, normal_direction)

        # If the direction is negative, adjust the angle to reflect a clockwise sweep past 180 degrees
        if direction < 0:
            angle_rad = 2 * np.pi - angle_rad

        # Convert radians to degrees
        angle_deg = np.degrees(angle_rad)

        return angle_rad

def main(args=None):
    rclpy.init(args=args)
    node = PendulumTFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


class Plotter:
    def __init__(self, times, velocities):
        self.times = times
        self.velocities = velocities

        # Set up the figure and axis
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-2, 2)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Cart Velocity (m/s)')

        # Start the animation
        self.ani = FuncAnimation(self.fig, self.update, self.data_gen, blit=False, interval=100, repeat=False)
        plt.show(block=False)

    def data_gen(self):
        while True:
            yield self.times, self.velocities

    def update(self, data):
        times, velocities = data
        self.line.set_data(times, velocities)
        self.ax.set_xlim(max(0, times[-1] - 10), times[-1])
        self.ax.set_ylim(min(velocities) - 0.1, max(velocities) + 0.1)
        self.ax.figure.canvas.draw()
        return self.line,

    def update_plot(self):
        self.ani.event_source.stop()
        self.ani = FuncAnimation(self.fig, self.update, self.data_gen, blit=False, interval=100, repeat=False)
        plt.pause(0.001)

class AngleTracker:
    def __init__(self):
        self.previous_angle = 0
        self.total_rotation = 0

    def update_angle(self, new_angle):
        delta_angle = new_angle - self.previous_angle
        
        # Check for wrapping
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360

        self.total_rotation += delta_angle
        self.previous_angle = new_angle

        # Return the total rotation to keep track of how many times and in which direction zero has been crossed
        return self.total_rotation