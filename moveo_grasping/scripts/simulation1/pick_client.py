#!/usr/bin/env python

import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler 

from actionlib import SimpleActionClient

import numpy as np
from std_srvs.srv import Empty

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


def pick():
	pick_g = PickUpPoseGoal()
	rospy.init_node('pick_client', anonymous=True)

	rospy.loginfo("Waiting for /pickup_pose AS...")
	pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)

	time.sleep(1.0)
	if not pick_as.wait_for_server(rospy.Duration(20)):
	  rospy.logerr("Could not connect to /pickup_pose AS")
	  exit()

	rospy.loginfo("Setting cube pose based on ArUco detection")
	#pick_g.object_pose.pose.position.x = -0.02
	#pick_g.object_pose.pose.position.y = 0.50
	pick_g.object_pose.pose.position.x = -0.35
	pick_g.object_pose.pose.position.y = 0.35
		
	pick_g.object_pose.pose.position.z = 0.34

	pick_g.object_pose.header.frame_id = "base_link" 
	pick_g.object_pose.header.stamp = rospy.Time.now()
	    
	quat = quaternion_from_euler(0.0, 0.0, 0.0) # roll, pitch, yaw
	pick_g.object_pose.pose.orientation = Quaternion(*quat.tolist())

	rospy.loginfo("Gonna pick:" + str(pick_g))
	pick_as.send_goal_and_wait(pick_g)
	rospy.loginfo("Done!")

	result = pick_as.get_result()

	if str(moveit_error_dict[result.error_code]) != "SUCCESS":
	  rospy.logerr("Failed to pick, not trying further")

def place():
	pick_g = PickUpPoseGoal()
	pick_g.object_pose.pose.position.x = 0.35
	pick_g.object_pose.pose.position.y = 0.35
	pick_g.object_pose.pose.position.z = 0.28

	pick_g.object_pose.header.frame_id = "base_link" 
	pick_g.object_pose.header.stamp = rospy.Time.now()
	    
	quat = quaternion_from_euler(0.0, 0.0, -0.785) # roll, pitch, yaw
	pick_g.object_pose.pose.orientation = Quaternion(*quat.tolist())
	rospy.loginfo("Waiting for /place_pose AS...")
	place_as = SimpleActionClient('/place_pose', PickUpPoseAction)
	time.sleep(1.0)
	if not place_as.wait_for_server(rospy.Duration(20)):
	  rospy.logerr("Could not connect to /place_pose AS")
	  exit()

	place_as.send_goal_and_wait(pick_g)
	rospy.loginfo("Done!")

pick()
#place()
