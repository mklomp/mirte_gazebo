#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from mirte_msgs.msg import *
from mirte_msgs.srv import *

import functools
import math
import sys

pub_left = speed_pub_left = pub_right = speed_pub_right = {}

last_data_left = 0
last_data_right = 0


def get_data_left(req):
    return GetEncoderResponse(last_data_left)


def get_data_right(req):
    return GetEncoderResponse(last_data_right)


def map_rad_to_ticks(rad):
    return int(rad / (2 * math.pi) * ticks)


last_value_left = last_value_right = 0


def calculate_difference(data):
    global last_value_left, last_value_right
    left_value = map_rad_to_ticks(data.position[0])
    right_value = map_rad_to_ticks(data.position[1])
    diff_left = int(abs(last_value_left - left_value))
    diff_right = int(abs(last_value_right - right_value))
    last_value_left = left_value
    last_value_right = right_value
    return [diff_left, diff_right]


def calculateEncoder(data, differences):
    if bidirectional:
        left_value = data.position[0]
        right_value = data.position[1]
        return [map_rad_to_ticks(left_value), map_rad_to_ticks(right_value)]
    else:
        return [last_data_left + differences[0], last_data_right + differences[1]]


def callback(data):
    global last_data_left, last_data_right
    if len(data.position) != 2:
        return
    differences = calculate_difference(data)
    [left, right] = calculateEncoder(data, differences)
    left_message = Encoder()
    right_message = Encoder()
    left_message.header = right_message.header = data.header
    left_message.value = left
    right_message.value = right
    pub_left.publish(left_message)
    pub_right.publish(right_message)
    last_data_left = left
    last_data_right = right


last_speed_left_count = 0


def publish_speed_data_left(self, event=None):
    global last_speed_left_count
    encoder = Encoder()
    # encoder.header = self.get_header()
    encoder.value = last_value_left - last_speed_left_count
    if not bidirectional:
        encoder.value = abs(encoder.value)
    last_speed_left_count = last_value_left
    speed_pub_left.publish(encoder)


last_speed_right_count = 0


def publish_speed_data_right(self, event=None):
    global last_speed_right_count
    encoder = Encoder()
    # encoder.header = self.get_header()
    encoder.value = last_value_right - last_speed_right_count
    if not bidirectional:
        encoder.value = abs(encoder.value)
    last_speed_right_count = last_value_right
    speed_pub_right.publish(encoder)


def listener():
    global pub_left, speed_pub_left, pub_right, speed_pub_right
    rospy.init_node(f"encoderConverter", anonymous=True)
    pub_left = rospy.Publisher("/mirte/encoder/left", Encoder, queue_size=1, latch=True)
    srv_left = rospy.Service("/mirte/get_encoder_left", GetEncoder, get_data_left)
    speed_pub_left = rospy.Publisher(
        "/mirte/encoder_speed/left", Encoder, queue_size=1, latch=True
    )
    pub_right = rospy.Publisher(
        "/mirte/encoder/right", Encoder, queue_size=1, latch=True
    )
    srv_right = rospy.Service("/mirte/get_encoder_right", GetEncoder, get_data_right)
    speed_pub_right = rospy.Publisher(
        "/mirte/encoder_speed/right", Encoder, queue_size=1, latch=True
    )
    rospy.Timer(rospy.Duration(1.0 / 10.0), publish_speed_data_left)
    rospy.Timer(rospy.Duration(1.0 / 10.0), publish_speed_data_right)

    rospy.Subscriber(f"/mirte/joint_states", JointState, callback)
    rospy.spin()


ticks = 540
bidirectional = True

if __name__ == "__main__":
    if len(sys.argv) == 3:
        ticks = eval(sys.argv[1])
        bidirectional = eval(sys.argv[2])
    listener()
