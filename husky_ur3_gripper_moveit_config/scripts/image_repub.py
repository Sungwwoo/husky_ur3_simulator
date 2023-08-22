#! /usr/bin/env python3

import sys
import rospy
import tf2_ros
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image

img = Image()
lbl = Image()


def RepubLabel():
    global lbl
    try:
        new_lbl = rospy.wait_for_message("/seg_label_refined", Image, timeout=rospy.Duration(secs=1))
    except rospy.ROSException:
        pub_lbl.publish(lbl)
        return
    lbl = new_lbl
    pub_lbl.publish(lbl)


def RepubImage():
    global img
    try:
        new_img = rospy.wait_for_message("/seg_image_refined", Image, timeout=rospy.Duration(secs=1))
    except rospy.ROSException:
        pub_img.publish(img)
        return
    img = new_img
    pub_img.publish(img)


if __name__ == "__main__":

    rospy.init_node("image_republisher", anonymous=True, disable_signals=True)
    pub_lbl = rospy.Publisher("/object_mask", Image, queue_size=10)
    pub_img = rospy.Publisher("/object_mask_image", Image, queue_size=10)

    while not rospy.is_shutdown():
        RepubLabel()
        RepubImage()
