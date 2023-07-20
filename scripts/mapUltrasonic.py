#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range, LaserScan
import functools
import math
# def talker():
#     pub = rospy.Publisher('/mirte/left/ultra', Range, queue_size=10)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    mess = Range()
    mess.radiation_type = mess.ULTRASOUND
    mess.min_range = 0.01
    mess.max_range = 5
    data.ranges = list(filter(lambda dist: dist < data.range_max, data.ranges))
    print(data.ranges)
    if(len(data.ranges) == 0) :
        mess.range = math.inf
    else:
        mess.range = functools.reduce(lambda x, y: x if x < y else y, data.ranges)
    pub.publish(mess)

def listener():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ultrasonicConverter', anonymous=True)

    rospy.Subscriber("/mirte/scan2", LaserScan, callback)
    pub = rospy.Publisher('/mirte/left/ultra', Range, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


