cmake_minimum_required(VERSION 3.1.3)
project(husky_ur3_gripper_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
)

catkin_package(
  CATKIN_DEPENDS roscpp
  pcl_conversions
  pcl_ros
  moveit_core
  moveit_ros_planning_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
