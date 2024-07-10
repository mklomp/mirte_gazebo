import rclpy
from rclpy.node import Node

import rclpy.node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.last_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        z = 0
        for x in msg.data:
            z += x
        diff = self.get_clock().now() - self.last_time
        self.get_logger().info(f'I heard: "{z}" {diff.nanoseconds/1000_000_000}' )
        self.last_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()