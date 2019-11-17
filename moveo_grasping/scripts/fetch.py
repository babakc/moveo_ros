#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

pose = group.get_current_pose()
group.set_goal_tolerance(0.005)
group.set_planner_id("RRTConfigDefault")
group.set_planning_time(50.0)
group.set_num_planning_attempts(100)
#group.allow_replanning(True)

group.execute(wait = True)

#print group

"""
pose_target = geometry_msgs.msg.Pose()

pose_target.position.x = 0.44
pose_target.position.y = 0.0
pose_target.position.z = 0.33

global roll, pitch, yaw
orientation_q = pose.pose.orientation
orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
print roll
print pitch
print yaw
print pose

orient = quaternion_from_euler (1.57, 0, 1.57)
pose_target.orientation.x = orient[0]
pose_target.orientation.y = orient[1]
pose_target.orientation.z = orient[2]
pose_target.orientation.w = orient[3]


#group.set_pose_target(pose_target)

#plan1 = group.plan()

#group.go(wait=True)

#rospy.sleep(5)

moveit_commander.roscpp_shutdown()


"""
