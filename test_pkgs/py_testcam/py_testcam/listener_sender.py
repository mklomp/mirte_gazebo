import rclpy
from rclpy.node import Node

import rclpy.node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from copy import deepcopy
import math
import time


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.last_time = self.get_clock().now()
        self.time = time.time()
        self.subscription = self.create_subscription(
            Image, "/camera/color/image_raw", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Image, "topic", 10)

    def listener_callback(self, msg: Image):
        z = 0
        # for x in msg.data:
        #     z += x
        msg2 = msg
        print(len(msg2.data))
        for x in range(0, 10000):  # len(msg2.data)):
            msg2.data[x] = 0  # min( msg2.data[x] +5, 255)
            # print(x, msg2.data[x])

        self.publisher_.publish(msg2)
        diff = self.get_clock().now() - self.last_time
        diff2 = time.time() - self.time
        s = diff.nanoseconds / 1000_000_000
        self.get_logger().info(f'I heard: "{z}" {s} {1/s} {diff2}, {1/diff2}')
        self.last_time = self.get_clock().now()
        self.time = time.time()


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
