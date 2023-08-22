#! /usr/bin/env python3

import sys
import rospy
import tf2_ros
import numpy as np
import cv2
import cv_bridge
import random
import pyrealsense2 as rs
from moveit_msgs.msg import DisplayTrajectory
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, Vector3, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from tf import TransformBroadcaster
from math import pi, sqrt, atan
from std_msgs.msg import String, Float64, Float64MultiArray, Header, Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from moveit_commander.conversions import pose_to_list
from ar_track_alvar_msgs.msg import AlvarMarkers
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


can_position = [-12.8, 26.66, 0.46]
spawnPoint = [-17, 26, 0.14]

z_offset = 0.1651 + 0.03282
reach = 0.5

# move_group configuration
robot = RobotCommander()
scene = PlanningSceneInterface()

group_name = "ur5_manipulator"
group = MoveGroupCommander(group_name)
# Must be specified for regular manipulation speed
group.set_max_velocity_scaling_factor(1.0)
group.set_max_acceleration_scaling_factor(1.0)

# Publishers
pub_display_trajectory = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
pub_nav_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
pub_grip_joint = rospy.Publisher("/rh_p12_rn_position/command", Float64, queue_size=10)
pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
pub_state = rospy.Publisher("/process_state", Int16, queue_size=10)
# move_base service client
client = SimpleActionClient("move_base", MoveBaseAction)

planning_frame = group.get_planning_frame()

# Global Variables
targetFound = False
gripperJointVal = 0.0


depth = np.ndarray((480, 640))
bridge = cv_bridge.CvBridge()


def ConstructTF(parentFrame, frameName, translation, rotation):
    br = TransformBroadcaster()
    trans = Transform(translation=translation, rotation=rotation)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = parentFrame
    trans_stamped = TransformStamped(header, frameName, trans)
    br.sendTransformMessage(trans_stamped)


def GetIntrinsicMatrix():
    # camInfo = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
    camInfo = rospy.wait_for_message("/g_camera/color/camera_info", CameraInfo)
    K = np.array(camInfo.K).reshape(3, 3)
    return K


def GetDepthData():
    for i in range(0, len(depth)):
        for j in range(0, len(depth[i])):
            depth[i][j] = 0
    depth_msg = rospy.wait_for_message("/g_camera/depth/image_raw", Image)
    depth_cv = bridge.imgmsg_to_cv2(depth_msg)
    # cv2.imshow("", depth_cv)
    # cv2.waitKey(0)
    converted = np.array(depth_cv)

    for i in range(0, len(converted)):
        for j in range(0, len(converted[i])):
            depth[i][j] += converted[i][j]

    return depth


def GetTF(target_frame, source_frame):

    while True:
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(1)
            continue


def cbGPoseMarker(data):
    global targetFound
    global targetPosition
    global gripperJointVal
    if data.markers == []:
        return
    else:
        for marker in data.markers:
            if marker.id == 0:
                targetFound = True
                targetPosition = GetTF("odom", "ar_marker_g_0")
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


def CalcDistance(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def GetLocation(target_frame, source_frame):
    trans = GetTF(target_frame, source_frame)
    return trans.transform.translation.x, trans.transform.translation.y


def MoveBackward(dist=0):
    print("Moving Backward")
    start_x, start_y = GetLocation("odom", "base_link")
    cmd = Twist()
    cmd.linear.x = -0.1
    current_x, current_y = GetLocation("odom", "base_link")
    pub_cmd.publish(cmd)
    while CalcDistance(start_x, start_y, current_x, current_y) < dist:
        pub_cmd.publish(cmd)
        current_x, current_y = GetLocation("odom", "base_link")
        continue

    cmd.linear.x = 0
    pub_cmd.publish(cmd)
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


def rotm2quat(rotm):
    epsilon = 1e-12
    quat = np.ndarray((4, 1))
    tr = rotm[0][0] + rotm[1][1] + rotm[2][2]
    if tr > epsilon:
        quat[0][0] = 0.5 * np.sqrt(tr + 1)
        s_inv = 1 / (quat[0][0] * 4)

        quat[1][0] = (rotm[2][1] - rotm[1][2]) * s_inv
        quat[2][0] = (rotm[0][2] - rotm[2][0]) * s_inv
        quat[3][0] = (rotm[1][0] - rotm[0][1]) * s_inv

    else:

        if rotm[0][0] > rotm[1][1] and rotm[0][0] > rotm[2][2]:
            quat[1][0] = 0.5 * np.sqrt(rotm[0][0] - rotm[1][1] - rotm[2][2] + 1)
            s_inv = 1 / (quat[1][0] * 4)
            quat[0][0] = (rotm[2][1] - rotm[1][2]) * s_inv
            quat[2][0] = (rotm[1][0] - rotm[0][1]) * s_inv
            quat[3][0] = (rotm[2][0] - rotm[0][2]) * s_inv

        elif rotm[1][1] > rotm[2][2]:
            quat[2][0] = 0.5 * np.sqrt(rotm[1][1] - rotm[2][2] - rotm[0][0] + 1)
            s_inv = 1 / (quat[2][0] * 4)
            quat[0][0] = (rotm[0][2] - rotm[2][0]) * s_inv
            quat[1][0] = (rotm[1][0] - rotm[0][1]) * s_inv
            quat[3][0] = (rotm[2][1] - rotm[1][2]) * s_inv

        else:
            quat[3][0] = 0.5 * np.sqrt(rotm[2][2] - rotm[0][0] - rotm[1][1] + 1)
            s_inv = 1 / (quat[3][0] * 4)
            quat[0][0] = (rotm[1][0] - rotm[0][1]) * s_inv
            quat[1][0] = (rotm[2][0] - rotm[0][2]) * s_inv
            quat[2][0] = (rotm[2][1] - rotm[1][2]) * s_inv

    return quat


if __name__ == "__main__":

    roscpp_initialize(sys.argv)
    rospy.init_node("Manipulation_test_node", anonymous=True, disable_signals=True)

    # tf listner
    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    rospy.loginfo("Reference frame: %s" % planning_frame)

    eef_link = group.get_end_effector_link()
    rospy.loginfo("End effector: %s" % eef_link)

    FindTarget()

    trans = GetTF("odom", "ur5_base_link")
    base_position = GetTF("odom", "base_link")
    rospy.loginfo("Calculating Approach Point")
    print("Current Base Odom: ", base_position.transform.translation.x, base_position.transform.translation.y)

    # xf, yf, zf = (
    #     targetPosition[0] - (spawnPoint[0] + base_position.transform.translation.x),
    #     targetPosition[1] - (spawnPoint[1] + base_position.transform.translation.y),
    #     targetPosition[2],
    # )
    xf, yf, zf = (targetPosition.transform.translation.x, targetPosition.transform.translation.y, targetPosition.transform.translation.z)

    xi, yi, zi = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z + z_offset
    print("Coke can location: ", xf, yf)

    totalDistance = sqrt((xi - xf) ** 2 + (yi - yf) ** 2)
    approachDistance = sqrt(reach**2 - (zf - zi) ** 2)
    distRatio = approachDistance / (totalDistance - approachDistance)
    targetX = (approachDistance * xi + (totalDistance - approachDistance) * xf) / totalDistance
    targetY = (approachDistance * yi + (totalDistance - approachDistance) * yf) / totalDistance
    theta = atan((yf - yi) / (xf - xi))
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
    manip_result = group.go(group.get_named_target_values("default"), wait=False)

    client.wait_for_result()
    # Grasp

    group.go(group.get_named_target_values("table_top_view"))

    state = Int16()
    state.data = 1
    pub_state.publish(state)

    print("Waiting for pose goal to be generated...")
    while rospy.wait_for_message("process_state", Int16).data != 2:
        continue

    input("press enter to continue")
    eef_target = GetTF("base_link", "target_pose")

    # # pre-grasp
    # pose_goal = Pose()
    # pose_goal.orientation.x = eef_target.transform.rotation.x
    # pose_goal.orientation.y = eef_target.transform.rotation.y
    # pose_goal.orientation.z = eef_target.transform.rotation.z
    # pose_goal.orientation.w = eef_target.transform.rotation.w
    # pose_goal.position.x = eef_target.transform.translation.x - 0.09
    # pose_goal.position.y = eef_target.transform.translation.y
    # pose_goal.position.z = eef_target.transform.translation.z

    # group.set_pose_target(pose_goal)

    # plan = group.go(wait=True)

    # input("press enter to continue")
    # grasp
    pose_goal = Pose()
    pose_goal.orientation.x = eef_target.transform.rotation.x
    pose_goal.orientation.y = eef_target.transform.rotation.y
    pose_goal.orientation.z = eef_target.transform.rotation.z
    pose_goal.orientation.w = eef_target.transform.rotation.w
    pose_goal.position.x = eef_target.transform.translation.x
    pose_goal.position.y = eef_target.transform.translation.y
    pose_goal.position.z = eef_target.transform.translation.z

    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)

    input("press enter to continue")
    gripper_joint = Float64()
    gripper_joint.data = 0.5
    pub_grip_joint.publish()
    rospy.sleep(1)

    input("press enter to continue")
    group.go(group.get_named_target_values("default"), wait=False)

    MoveBackward(0.3)

    rand_x = random.randrange(-120, 120) / 100.0
    rand_y = random.randrange(-120, 120) / 100.0

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
    exit()
