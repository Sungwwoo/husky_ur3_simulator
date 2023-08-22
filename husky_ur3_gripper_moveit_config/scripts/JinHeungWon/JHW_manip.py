#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
import sys
import rospy
import copy, math
import threading
import time
import tf, signal

from threading import Thread
import multiprocessing
from multiprocessing import Process
from math import pi, radians, degrees, atan2, sqrt
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_commander.conversions import pose_to_list, list_to_pose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pi
from ar_track_alvar_msgs.msg import AlvarMarkers

import numpy as np
from numpy import linalg


import numpy as np
import tf.transformations as tf
from math import *
import cmath
from geometry_msgs.msg import Pose, Quaternion


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def ar_position(msg):

    global ar_x
    global ar_y
    global ar_z

    global ar_x_1
    global ar_y_1
    global ar_z_1

    ar_topic_pose = Pose()
    ar_topic_pose_1 = Pose()
    try:
        ar_topic_pose.position.x = msg.markers[0].pose.pose.position.x
        ar_topic_pose.position.y = msg.markers[0].pose.pose.position.y
        ar_topic_pose.position.z = msg.markers[0].pose.pose.position.z
        ar_x = ar_topic_pose.position.x
        ar_y = ar_topic_pose.position.y
        ar_z = ar_topic_pose.position.z

        ar_topic_pose_1.position.x = msg.markers[1].pose.pose.position.x
        ar_topic_pose_1.position.y = msg.markers[1].pose.pose.position.y
        ar_topic_pose_1.position.z = msg.markers[1].pose.pose.position.z
        ar_x_1 = ar_topic_pose.position.x
        ar_y_1 = ar_topic_pose.position.y
        ar_z_1 = ar_topic_pose.position.z
    except:
        return


def jmove_to_pose_goal(pose_goal):

    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    # tf_display_position = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
    # tf_display_orientation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
    # ii = 0
    # while ii < 5:
    #    ii += 1
    #    br = tf.TransformBroadcaster()
    #    br.sendTransform(
    #        tf_display_position,
    #        tf_display_orientation,
    #        rospy.Time.now(),
    #        "Target_pose",
    #        "base_link")
    #    rospy.sleep(1)


def move_Joint(q1, q2, q3, q4, q5, q6):
    joint_goal = move_group.get_current_joint_values()

    joint_goal_list = [q1, q2, q3, q4, q5, q6]

    # 매니퓰레이터 관절 value 설정
    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]
    # 제어시작
    move_group.go(joint_goal, wait=False)


def cartesian_path_planner(a, b):
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z += b  # First move up (z)
    wpose.position.y += a
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.15, 0.0)  # waypoints to follow  # eef_step  # jump_threshold

    move_group.execute(plan, wait=True)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # GROUP_NAME_GRIPPER = "NAME OF GRIPPER"
    roscpp_initialize(sys.argv)
    rospy.init_node("control_Husky_UR3", anonymous=True)
    robot = RobotCommander()
    scene = PlanningSceneInterface()

    ##모바일 파트 관련 변수 선언
    x = 0.0
    y = 0.0
    theta = 0.0

    ## 매니퓰레이터 변수 선언

    group_name = "ur3_manipulator"
    move_group = MoveGroupCommander(group_name)
    FIXED_FRAME = "world"

    display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)

    # ar마커 변수 선언
    ar_x = 0
    ar_y = 0
    ar_z = 0

    # 사용자입력 : 목표 eef 위치 지정 (x,y,z,r,p,yaw)
    goal_eef = [0, 0, 0, 0, 0, 0, 1]
    goal_eef_quat = list_to_pose(goal_eef)

    # ar 마커 및 eef pose data 선언.
    ar_pose = Pose()
    eef_pose = Pose()

    sub1 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_position)

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    speed = Twist()
    rospy.sleep(2)

    r = rospy.Rate(20)

    ur_reco_reach = 0.5

    prev_t = time.time()

    ar_pose = Pose()
    ar_pose.position.x = 0.7
    ar_pose.position.y = 0.0
    ar_pose.position.z = 0.6
    ar_pose.orientation.x = 0.707
    ar_pose.orientation.y = 0.0
    ar_pose.orientation.z = 0.707
    ar_pose.orientation.w = 0.0
    rospy.sleep(0.1)
    jmove_to_pose_goal(ar_pose)
    input()

    while not rospy.is_shutdown():

        cartesian_path_planner(0.1, -0.1)

        # input()
        rospy.sleep(0.1)
        cartesian_path_planner(-0.2, 0)

        # input()
        rospy.sleep(0.1)
        cartesian_path_planner(0, 0.2)

        # input()
        rospy.sleep(0.1)
        cartesian_path_planner(0.2, 0)

        # input()
        rospy.sleep(0.1)
        cartesian_path_planner(-0.1, -0.1)

        # input()

        # ar_pose.position.x = 0.6
        # ar_pose.position.y = -0.2
        # ar_pose.position.z = 0.5
        # rospy.sleep(0.1)
        # jmove_to_pose_goal(ar_pose)
        # input()
#
# ar_pose.position.x = 0.6
# ar_pose.position.y = 0.2
# ar_pose.position.z = 0.5
# rospy.sleep(0.1)
# jmove_to_pose_goal(ar_pose)
# input()
#
# ar_pose.position.x = 0.6
# ar_pose.position.y = 0.2
# ar_pose.position.z = 0.8
# rospy.sleep(0.1)
# jmove_to_pose_goal(ar_pose)
# input()
#
# ar_pose.position.x = 0.6
# ar_pose.position.y = -0.2
# ar_pose.position.z = 0.8
# rospy.sleep(0.1)
# jmove_to_pose_goal(ar_pose)
# input()
#
#
