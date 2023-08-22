#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
import sys
import rospy
import copy, math
import threading
import time
import tf

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
    ar_topic_pose = Pose()
    try:
        ar_topic_pose.position.x = msg.markers[0].pose.pose.position.x
        ar_topic_pose.position.y = msg.markers[0].pose.pose.position.y
        ar_topic_pose.position.z = msg.markers[0].pose.pose.position.z
        ar_x = ar_topic_pose.position.x
        ar_y = ar_topic_pose.position.y
        ar_z = ar_topic_pose.position.z
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


def jmove_to_joint_goal(joint_goal):
    move_group.go(joint_goal, wait=False)


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


def get_TF(a, b):
    end_flag = 0
    listener = tf.TransformListener()
    while end_flag == 0:
        try:
            (trans, rot) = listener.lookupTransform(a, b, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        end_flag = 1

    return trans, rot


def move_base(a, b):

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    speed = Twist()
    r = rospy.Rate(4)

    goal = Point()
    goal.x = a
    goal.y = b

    arrival_radius = 0.3

    while (goal.x - x) ** 2 + (goal.y - y) ** 2 >= arrival_radius**2:
        # while abs(goal.x-x) >0.1 or abs(goal.y-y) >0.1 or abs(angle_to_goal-theta) >0.1 : #가까의 범위가 0.3이내로 들어오면 break.

        inc_x = goal.x - x
        inc_y = goal.y - y
        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 5 * pi / 180:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
            if abs(angle_to_goal - theta) < 5 * pi / 180:  # 0.5이내로 들어오면 속도를 매우 줄여서 목표점을 지나쳐버리는 일이 없도록함.
                speed.angular.z = 0.03
                speed.linear.x = 0.0

        else:
            speed.linear.x = 0.2
            speed.angular.z = 0.0
            if abs(goal.x - x) < 0.3 and abs(goal.y - y) < 0.3:  # x,y val이 0.3이내로 들어오면 속도 매우 줄임.
                speed.angular.x = 0.05
                speed.angular.z = 0.0

        # print(goal.x-x, goal.y-y, angle_to_goal-theta)

        pub.publish(speed)
        # r.sleep()

    final_angle_to_goal = 0
    while abs(final_angle_to_goal - theta) > 0.02:
        if abs(final_angle_to_goal - theta) > 0.3:
            speed.linear.x = 0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0
            speed.angular.z = 0.1

        pub.publish(speed)
        r.sleep()

    print("mobile robot movement complete!")

    return x, y


def Grasp_object(x_dir, y_dir, z_dir):

    current_mobile_x, current_mobile_y = move_base(x_dir - 0.5, y_dir)
    # z_path_planner(0.1)
    print("Grasping is ready to start!, press enter..!")
    input()
    curr_pose = move_group.get_current_pose().pose

    x_distance = current_mobile_x + curr_pose.position.x - x_dir
    y_distance = current_mobile_y + curr_pose.position.y - y_dir
    z_distance = curr_pose.position.z - z_dir

    print(curr_pose.position.x)
    print("x_dir =", x_dir, "y_dir=", y_dir, "z_dir=", z_dir)

    print("x =", x_distance, "y=", y_distance, "z=", z_distance)
    # y_path_planner(-y_distance)
    # x_path_planner(-x_distance)
    # z_path_planner(-z_distance)
    plan, fraction = cartesian_path(-x_distance, -y_distance, -z_distance)
    move_group.execute(plan, wait=True)

    rospy.sleep(3)
    (result_xyz, result_rot) = get_TF("/odom", "ee_link")
    print("xyz_result=", result_xyz[0], result_xyz[1], result_xyz[2])
    print("Grasping complete!, Go to home pose..!")


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

    # vector_x = ar_tr[0] - eef_tr[0]
    # vector_y = ar_tr[1] - eef_tr[1]

    # angle_to_goal = atan2(vector_y,vector_x)
    # distance_to_goal_eef = sqrt(vector_x**2 + vector_y**2)

    # ar 마커 및 eef pose data 선언.
    ar_pose = Pose()
    eef_pose = Pose()

    sub1 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_position)

    sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # rospy.wait_for_message('ar_pose_marker', AlvarMarkers)

    speed = Twist()
    # move_Joint(0,0,0,0,0,0)
    # 목표eef의 방향으로 x 미터 앞까지 모바일 플랫폼을 직진
    # move_Joint(pi/2,0,-pi/3,0,pi/2,0)
    rospy.sleep(2)

    r = rospy.Rate(20)

    ur_reco_reach = 0.5
    print("press enter to start")
    prev_t = time.time()

    cnt = 0

    ar_pose = Pose()
    ar_pose.position.x = 0.7
    ar_pose.position.y = 0.0
    ar_pose.position.z = 0.6
    ar_pose.orientation.x = 0.707
    ar_pose.orientation.y = 0.0
    ar_pose.orientation.z = 0.707
    ar_pose.orientation.w = 0.0
    print(ar_pose.position.x)
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

        dist_x = 0
        prev_x = x
        dist_to_go = 1.5  # 3m
        # dist_ar = 0

        while dist_to_go - dist_x > 0.05:

            # ee_state = move_group.get_current_pose()
            # ee_x = ee_state.pose.position.x

            # dist_ar = abs(ar_x-ee_x)

            curr_x = x
            dist_x = curr_x - prev_x
            last_dist = dist_to_go - dist_x
            desired_speed = 0.2 / (np.exp(-12 * last_dist + 6) + 1) + 0.05
            speed.linear.x = desired_speed
            speed.angular.z = 0.0

            # print(ar_x,ee_x)
            print("desired speed:", desired_speed)
            print("distance:", last_dist)

            pub.publish(speed)
            r.sleep()
        # input()
        rospy.sleep(0.1)

        cartesian_path_planner(0, 0.2)

        # input()
        rospy.sleep(0.1)
        cartesian_path_planner(0.2, 0)

        # input()
        rospy.sleep(0.1)
        cartesian_path_planner(-0.1, -0.1)

        input()
