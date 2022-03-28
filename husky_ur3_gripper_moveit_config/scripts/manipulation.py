#! /usr/bin/env python3

import sys
import rospy
import tf2_ros
import numpy as np
import cv2
import cv_bridge
import random
from moveit_msgs.msg import DisplayTrajectory
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi, sqrt, atan
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from moveit_commander.conversions import pose_to_list
from ar_track_alvar_msgs.msg import AlvarMarkers
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


targetPosition = [-12.8, 26.66, 0.46]
spawnPoint = [-17, 26, 0.14]

z_offset = 0.1651 + 0.03282
reach = 0.8

# move_group configuration
robot = RobotCommander()
scene = PlanningSceneInterface()

group_name = "ur3_manipulator"
group = MoveGroupCommander(group_name)
# Must be specified for regular manipulation speed
group.set_max_velocity_scaling_factor(1.0)
group.set_max_acceleration_scaling_factor(1.0)


# Publishers
pub_display_trajectory = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
pub_nav_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
pub_grip_joint = rospy.Publisher("/rh_p12_rn_position/command", Float64, queue_size=10)
pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# move_base service client
client = SimpleActionClient("move_base", MoveBaseAction)

planning_frame = group.get_planning_frame()

# Global Variables+
targetFound = False
targetPose = PoseStamped()
gripperJointVal = 0.0


def cbGPoseMarker(data):
    global targetFound
    global targetPose
    global gripperJointVal
    if data.markers == []:
        return
    else:
        for marker in data.markers:
            if marker.id == 0:
                targetFound = True
                targetPose = marker.pose
                gripperJointVal = 0.5  # Assuming Predefined
                return


def FindTarget():
    sub_g_pose_marker = rospy.Subscriber("/ar_pose_marker_g", AlvarMarkers, cbGPoseMarker)
    # Find target(ar_marker with id 0)
    rospy.loginfo("Finding Target")

    group.go(group.get_named_target_values("front_view"), wait=True)
    if not targetFound:
        current_joint = group.get_current_joint_values()
        joint_goal = current_joint[:]
        joint_goal[4] -= pi / 2
        group.go(
            joint_goal,
            wait=True,
        )
    else:
        return
    if not targetFound:
        joint_goal[4] += pi
        group.go(joint_goal, wait=True)
    else:
        return

    if not targetFound:
        joint_goal[4] += pi / 2
        group.go(joint_goal, wait=True)
    else:
        return

    if not targetFound:
        rospy.loginfo("No Targets Found")
        group.go(group.get_named_target_values("default"))
        exit()


def GetTF(target_frame, source_frame):
    # tf listner
    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    while True:
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1)
            continue


def MoveBackward(dist=0):
    return


def MoveJoint(joint_goal_list=[]):

    joint_goal = [0 for i in range(6)]
    # Set joint goal
    joint_goal[0] = joint_goal_list[0]
    joint_goal[1] = joint_goal_list[1]
    joint_goal[2] = joint_goal_list[2]
    joint_goal[3] = joint_goal_list[3]
    joint_goal[4] = joint_goal_list[4]
    joint_goal[5] = joint_goal_list[5]

    # Execute
    group.go(joint_goal, wait=True)


if __name__ == "__main__":

    roscpp_initialize(sys.argv)
    rospy.init_node("Manipulation_test_node", anonymous=True, disable_signals=True)

    rospy.loginfo("Reference frame: %s" % planning_frame)

    eef_link = group.get_end_effector_link()
    rospy.loginfo("End effector: %s" % eef_link)

    FindTarget()

    trans = GetTF("odom", "ur3_base_link")
    base_position = GetTF("odom", "base_link")
    rospy.loginfo("Calculating Approach Point")
    print("Current Base Odom: ", base_position.transform.translation.x, base_position.transform.translation.y)

    xf, yf, zf = (
        targetPosition[0] - (spawnPoint[0] + base_position.transform.translation.x),
        targetPosition[1] - (spawnPoint[1] + base_position.transform.translation.y),
        targetPosition[2],
    )
    xi, yi, zi = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z + z_offset
    print("Coke can location: ", xf, yf)
    totalDistance = sqrt((xi - xf) ** 2 + (yi - yf) ** 2)
    approachDistance = sqrt(reach**2 - (zf - zi) ** 2)
    distRatio = approachDistance / (totalDistance - approachDistance)
    targetX = (approachDistance * xi + (totalDistance - approachDistance) * xf) / totalDistance
    targetY = (approachDistance * yi + (totalDistance - approachDistance) * yf) / totalDistance
    theta = atan((yi - yf) / (xi - xf))
    base_goal_quat = quaternion_from_euler(0, 0, theta)
    rospy.loginfo("Moving to Target...")

    # Find approaching direction
    print("Moving to ", targetX, targetY)
    base_goal = MoveBaseGoal()
    base_goal.target_pose.header.frame_id = "odom"
    base_goal.target_pose.header.stamp = rospy.Time.now()
    base_goal.target_pose.pose.position.x = targetX
    base_goal.target_pose.pose.position.y = targetY
    base_goal.target_pose.pose.position.z = 0
    base_goal.target_pose.pose.orientation.x = base_goal_quat[0]
    base_goal.target_pose.pose.orientation.y = base_goal_quat[1]
    base_goal.target_pose.pose.orientation.z = base_goal_quat[2]
    base_goal.target_pose.pose.orientation.w = base_goal_quat[3]
    client.send_goal(base_goal)
    group.go(group.get_named_target_values("default"), wait=False)

    client.wait_for_result()
    input("\nPress enter to continue\n")
    # Grasp

    current_pose = group.get_current_pose(end_effector_link=eef_link)
    print(current_pose)

    pose_goal = Pose()

    quat = quaternion_from_euler(pi, pi / 2, pi)
    pose_goal.orientation.x = current_pose.pose.orientation.x
    pose_goal.orientation.y = current_pose.pose.orientation.y
    pose_goal.orientation.z = current_pose.pose.orientation.z
    pose_goal.orientation.w = current_pose.pose.orientation.w

    pose_goal.position.x = current_pose.pose.position.x + 0.05
    pose_goal.position.y = current_pose.pose.position.y
    pose_goal.position.z = current_pose.pose.position.z

    group.set_pose_target(pose_goal)

    rospy.loginfo("Executing")
    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    # Grasp Object

    pub_grip_joint.publish(gripperJointVal)

    group.go(group.get_named_target_values("default"), wait=False)

    rand_x = random.randrange(-12, 12) / 10.0
    rand_y = random.randrange(-12, 12) / 10.0
    print("Moving to ", rand_x, rand_y)
    base_goal = MoveBaseGoal()
    base_goal.target_pose.header.frame_id = "odom"
    base_goal.target_pose.header.stamp = rospy.Time.now()
    base_goal.target_pose.pose.position.x = rand_x
    base_goal.target_pose.pose.position.y = rand_y
    base_goal.target_pose.pose.position.z = 0
    base_goal.target_pose.pose.orientation.x = 0
    base_goal.target_pose.pose.orientation.y = 0
    base_goal.target_pose.pose.orientation.z = 0
    base_goal.target_pose.pose.orientation.w = 1
    client.send_goal(base_goal)
