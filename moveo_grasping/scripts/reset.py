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
from std_srvs.srv import Empty, EmptyRequest

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

#rospy.sleep(3)
	
rospy.loginfo("Cleaning world objects")
# clean the scene
scene.remove_world_object("part")
scene.is_diff = True


# We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()

joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
joint_goal[3] = 0
joint_goal[4] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
#group.stop()
