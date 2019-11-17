#!/usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler 

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)


robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

group.set_goal_tolerance(0.01)
group.set_planner_id("RRTConfigDefault")
group.set_planning_time(50.0)
group.set_num_planning_attempts(50),
group.allow_replanning(True)
scene.is_diff = True


orientation = quaternion_from_euler(-1.57, 0, -0.785)

pose = geometry_msgs.msg.Pose()
pose.position.x = 0.30
pose.position.y = 0.30
pose.position.z = 0.23
pose.orientation.x = orientation[0]
pose.orientation.y = orientation[1]
pose.orientation.z = orientation[2]
pose.orientation.w = orientation[3]
group.set_pose_target(pose)

plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
