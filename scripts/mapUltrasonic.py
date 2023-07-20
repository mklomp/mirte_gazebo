#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range, LaserScan
from mirte_msgs.msg import *
from mirte_msgs.srv import *

import functools
import math
import sys

last_publish_value = {}


def get_data(req):
    return GetDistanceResponse(last_publish_value.range)


def callback(data):
    global last_publish_value
    mess = Range()
    mess.radiation_type = mess.ULTRASOUND
    mess.min_range = data.range_min
    mess.max_range = data.range_max
    mess.field_of_view = data.angle_max - data.angle_min
    mess.header = data.header
    data.ranges = list(filter(lambda dist: dist < data.range_max, data.ranges))
    if len(data.ranges) == 0:
        mess.range = math.inf
    else:
        mess.range = functools.reduce(lambda x, y: x if x < y else y, data.ranges)
    pub.publish(mess)
    last_publish_value = mess


def listener(direction):
    global pub
    rospy.init_node(f"ultrasonicConverter{direction}", anonymous=True)

    rospy.Subscriber(f"/mirte/scanSonar{direction}", LaserScan, callback)
    pub = rospy.Publisher(f"/mirte/distance/{direction}", Range, queue_size=10)
    s = rospy.Service(f"/mirte/get_distance_{direction}", GetDistance, get_data)
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Missing direction")
        quit(-1)
    listener(sys.argv[1])
