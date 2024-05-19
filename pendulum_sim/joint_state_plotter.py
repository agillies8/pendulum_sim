import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('joint_state_plotter')

        # Subscribe to the joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # List to store cart velocities for plotting
        self.cart_velocities = []
        self.times = []

        # Initialize the plotter
        self.plotter = Plotter(self.times, self.cart_velocities)

        self.get_logger().info('Joint State Plotter Node has been started.')

    def joint_state_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.get_logger().info(f'Received joint state message at time {current_time}')

        # Extract positions for 'cart' and 'arm'
        try:
            cart_index = msg.name.index('cart')
            arm_index = msg.name.index('arm')
            # self.cart_positions.append(msg.position[cart_index])
            # self.arm_positions.append(msg.position[arm_index])


            # Update the velocity list for plotting
            self.times.append(current_time)
            self.cart_velocities.append(msg.velocity[arm_index])
            self.plotter.update_plot()

        except ValueError as e:
            self.get_logger().warn(f'Joint not found: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

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