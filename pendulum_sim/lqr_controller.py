import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
import scipy.linalg

class InvertedPendulumController(Node):
    def __init__(self):
        super().__init__('inverted_pendulum_controller')

        # Subscribe to the state topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'pendulum_state',
            self.state_callback,
            10)

        # Publisher for the control input
        self.publisher = self.create_publisher(Float32, 'control_input', 10)

        # Define system parameters
        #These are taken from onshape: https://cad.onshape.com/documents/a25fd7446c6816d118f41bc7/v/ea0ee1229d9cc72d1dc7ae85/e/14f430cb89b5dbb9a7de108a
        self.m = 0.238  # mass of the pendulum (kg)
        self.M = 0.383  # mass of the cart (kg)
        self.l = 0.250  # length of the pendulum (m)
        self.g = 9.81  # acceleration due to gravity (m/s^2)

        # Linearized system matrices
        self.A = np.array([[0, 1, 0, 0],
                          [0, 0, -self.m * self.g / (self.M + self.m), 0],
                          [0, 0, 0, 1],
                          [0, 0, self.g * (self.M + self.m) / self.l / (self.M + self.m), 0]])

        self.B = np.array([[0],
                          [1 / (self.M + self.m)],
                          [0],
                          [-1 / self.l / (self.M + self.m)]])

        # LQR weighting matrices
        Q = np.diag([1, 1, 10, 100])
        R = np.array([[0.01]])

        # Solve the Riccati equation
        P = scipy.linalg.solve_continuous_are(self.A, self.B, Q, R)

        # Compute the LQR gain
        self.K = np.dot(np.linalg.inv(R), np.dot(self.B.T, P))

        # Desired set point for the state vector
        self.x_ref = np.array([0.5, 0, 0, 0])

        self.get_logger().info('Inverted Pendulum Controller Node Started')

    def state_callback(self, msg):
        state = np.array(msg.data)

        # Ensure the state vector is of the correct shape
        if state.shape[0] != 4:
            self.get_logger().error('Received state vector of incorrect size')
            return

        # Compute the control input with respect to the set point
        error = state - self.x_ref
        u = -np.dot(self.K, error)

        # Publish the control input
        control_msg = Float32()
        control_msg.data = float(u)
        self.publisher.publish(control_msg)

        self.get_logger().info(f'Published control input: {control_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = InvertedPendulumController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
