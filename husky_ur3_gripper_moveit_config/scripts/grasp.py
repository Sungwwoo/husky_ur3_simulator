#! /usr/bin/env python3
import rospy


pub_grip_joint = rospy.Publisher("/rh_p12_rn_position/command", Float64, queue_size=10)
