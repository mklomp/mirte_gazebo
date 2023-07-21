#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from mirte_msgs.msg import Intensity, IntensityDigital
from std_msgs.msg import Header, Int32
from mirte_msgs.srv import *

"""
Node to transform an input Image topic into
an output grayscale Image topic.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class IR_sensor(object):
    def __init__(self, direction):
        self.cv_bridge = CvBridge()
        self.pub = rospy.Publisher(
            f"/mirte/intensity/{direction}", Intensity, queue_size=100
        )
        self.pubDig = rospy.Publisher(
            f"/mirte/intensity/{direction}_digital", IntensityDigital, queue_size=100
        )

        self.sub = rospy.Subscriber(
            f"/mirte/camera_ir_{direction}/image_raw",
            Image,
            self.image_cb,
            queue_size=5,
        )

        rospy.Service(
            f"/mirte/get_intensity_{direction}_digital",
            GetIntensityDigital,
            self.get_data_digital,
        )
        rospy.Service(
            f"/mirte/get_intensity_{direction}",
            GetIntensity,
            self.get_data_analog,
        )

    def get_data_analog(self, req):
        return GetIntensityResponse(self.last_analog.value)

    def get_data_digital(self, req):
        return GetIntensityDigitalResponse(self.last_digital.value)

    def get_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        return header

    def image_cb(self, img_msg):
        # Transform to cv2/numpy image
        img_in_cv2 = self.cv_bridge.imgmsg_to_cv2(
            img_msg, desired_encoding="passthrough"
        )
        if "rgb" in img_msg.encoding:
            gray_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_RGB2GRAY)
        elif "bgr" in img_msg.encoding:
            gray_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_BGR2GRAY)
        # Transform back to Image message]
        analog = np.average(gray_img)

        intensity = Intensity()
        intensity.header = self.get_header()
        intensity.value = int(analog)

        self.pub.publish(intensity)

        intensityDigital = IntensityDigital()
        intensityDigital.header = self.get_header()
        intensityDigital.value = bool(analog > 128)

        self.pubDig.publish(intensityDigital)
        self.last_analog = intensity
        self.last_digital = intensityDigital


direction = "left"
if __name__ == "__main__":
    argv = rospy.myargv(sys.argv)
    if len(argv) != 2:
        print("Missing direction, left or right")
        exit(0)
    direction = argv[1]
    rospy.init_node(f"ir_sensor_{direction}", anonymous=True)
    IR_sensor(direction)
    rospy.spin()
