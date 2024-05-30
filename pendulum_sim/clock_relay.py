import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import time

class ClockRelayNode(Node):

    def __init__(self):
        super().__init__('clock_relay_node')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Clock,
            'isaac_clock',
            self.clock_callback,
            10)
        
        # Publisher
        self.publisher = self.create_publisher(Clock, 'ros_clock', 10)

    def clock_callback(self, msg):
        relay_msg = Clock()
        relay_msg.clock.sec = msg.clock.sec
        relay_msg.clock.nanosec = msg.clock.nanosec
        self.get_logger().info(f'Published timestamp: {relay_msg.clock}')        
        time.sleep(10.0)
        self.publisher.publish(relay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClockRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
