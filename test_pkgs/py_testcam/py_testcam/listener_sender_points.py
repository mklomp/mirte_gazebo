import rclpy
from rclpy.node import Node

import rclpy.node
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from copy import deepcopy
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.last_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(PointCloud2, 'topic2', 10)


    def listener_callback(self, msg:PointCloud2):
        z = 0
        # msg.
        # for x in msg.data:
        #     z += x
        t2 = self.get_clock().now()
        msg2 = msg
        print(len(msg2.data))
        for x in range(1000):
            # print(x, msg2.data[x])
            msg2.data[x] = min( msg2.data[x] +5, 255)
        self.publisher_.publish(msg2)
        diff = self.get_clock().now() - self.last_time
        s = diff.nanoseconds/1000_000_000
        s2 = (self.get_clock().now() - t2).nanoseconds/1000_000_000
        self.get_logger().info(f'I heard: "{z}" {s} {1/s} {s2}' )
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