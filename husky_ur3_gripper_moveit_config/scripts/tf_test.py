#! /usr/bin/env python3

import rospy
import tf2_ros

if __name__ == "__main__":
    rospy.init_node("tf2_test_node", disable_signals=True)

    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("ur3_base_link", "shoulder_link", rospy.Time())
            print(trans)
            exit()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1)
            continue
