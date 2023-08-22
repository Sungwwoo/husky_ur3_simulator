#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


def cbLaser(scan):
    twist = Twist()
    length = len(scan.ranges)
    front = np.array(scan.ranges[length // 2 - 2 : length // 2 + 3])
    mean = np.mean(front)

    twist.linear.y, twist.linear.z = 0.0, 0.0
    twist.angular.x, twist.angular.y, twist.angular.z = 0.0, 0.0, 0.0
    if mean < 0.8:
        twist.linear.x = 0.0
    else:
        twist.linear.x = 0.2

    pub_cmd.publish(twist)


if __name__ == "__main__":
    rospy.init_node("front_laser_stop")

    sub_scan = rospy.Subscriber("/scan", LaserScan, cbLaser)

    pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
