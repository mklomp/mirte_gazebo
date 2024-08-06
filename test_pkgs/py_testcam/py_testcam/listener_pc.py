import rclpy
from rclpy.node import Node

import rclpy.node
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.last_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            PointCloud2, "/mirte/depth_camera/points", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, "/mirte/cmd_vel", 10)

    def listener_callback(self, msg: PointCloud2):
        z = 0
        msg2 = Twist()
        msg2.angular.z = -0.5
        msg2.linear.x = 0.1
        if msg.data[0] % 2 == 0:
            msg2.angular.z = 0.5
        self.publisher_.publish(msg2)
        diff = self.get_clock().now() - self.last_time
        self.get_logger().info(f'I heard: "{z}" {diff.nanoseconds/1000_000_000}')
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


if __name__ == "__main__":
    main()
