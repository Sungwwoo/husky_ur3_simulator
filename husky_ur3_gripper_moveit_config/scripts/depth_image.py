#! /usr/bin/env python3

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, PointCloud2

bridge = cv_bridge.CvBridge()


def cbDepth(points):
    print(points)
    return


def cbDepthImage(image):
    return


sub_depth = rospy.Subscriber("/h_d435/depth/h_points", PointCloud2, cbDepth)
sub_depth_image = rospy.Subscriber("/h_d435/depth/h_image_raw", Image, cbDepthImage)


if __name__ == "__main__":
    rospy.init_node("depth_image_test", disable_signals=True)
