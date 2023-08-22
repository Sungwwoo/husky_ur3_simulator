#! /usr/bin/env python3

import rospy
import sys
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import DisplayTrajectory
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize


if __name__ == "__main__":
    roscpp_initialize(sys.argv)
    rospy.init_node("set_default_pose", anonymous=True, disable_signals=True)
    rospy.sleep(5)
    # move_group configuration
    robot = RobotCommander()
    scene = PlanningSceneInterface()

    group_name = "ur5_manipulator"
    group = MoveGroupCommander(group_name)
    # Must be specified for regular manipulation speed
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)
    pub_display_trajectory = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)
    planning_frame = group.get_planning_frame()

    group.go(group.get_named_target_values("default"), wait=False)
