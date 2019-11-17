#!/usr/bin/python

import moveit_msgs.msg
import geometry_msgs.msg
import moveit_commander
import sys
import time
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_srvs.srv import Empty, EmptyRequest
from math import radians, pi

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_n_place', anonymous=True)
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm")
place_location = moveit_msgs.msg.PlaceLocation()
grasp = moveit_msgs.msg.Grasp()
now = rospy.Time.now()
clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
clear_octomap_srv.wait_for_service()
scene.is_diff = True

def createworld():
	rospy.sleep(3)
	
	rospy.loginfo("Cleaning world objects")
	# clean the scene
	scene.remove_world_object("table1")
	scene.remove_world_object("table2")
	scene.remove_world_object("part")
	    
	    # publish a demo scene
	p = PoseStamped()
	p.header.frame_id = '/base_link'
	p.header.stamp = rospy.Time.now()
	    
	p.pose.position.x = 0.0
	p.pose.position.y = 0.7
	p.pose.position.z = 0.15
	p.pose.orientation.w = 1.0
	#scene.add_box("table1", p, (0.3, 0.3, 0.3))
	p.pose.position.x = 0.7
	p.pose.position.y = 0.0
	p.pose.position.z = 0.15
	p.pose.orientation.w = 1.0
	#scene.add_box("table2", p, (0.3, 0.3, 0.3))
	p.pose.position.x = -0.35
	p.pose.position.y = 0.35
	p.pose.position.z = 0.30
    
	quat = quaternion_from_euler(0.0, 0.0, 0.0) # roll, pitch, yaw
	p.pose.orientation = Quaternion(*quat.tolist())

	scene.add_box("part", p, (0.03, 0.03, 0.15))
	rospy.loginfo("Added object to world")

def clearOctomap():
	clear_octomap_srv.call(EmptyRequest())

def setMoveGroupSettings():
	move_group.allow_replanning(True)
	move_group.set_goal_tolerance(0.5)
	move_group.set_planner_id("RRTConfigDefault")
	move_group.set_planning_time(50.0)
	move_group.set_num_planning_attempts(50),
	move_group.allow_replanning(True)


def preGraspPosture(): 
	pre_grasp_posture = JointTrajectory() 
	pre_grasp_posture.header.frame_id = "base_link" 
	pre_grasp_posture.header.stamp = now 
	pre_grasp_posture.joint_names = ["Gripper_Servo_Gear_Joint", "Gripper_Idol_Gear_Joint"]
	pos = JointTrajectoryPoint()
	pos.positions.append(0.0)
	pos.positions.append(0.0)
	pos.time_from_start = rospy.Duration(0.5)
	pre_grasp_posture.points.append(pos)
	return pre_grasp_posture

def graspPosture():
	grasp_posture = JointTrajectory()
	grasp_posture.header.frame_id = "base_link" 
	grasp_posture.header.stamp = now
	grasp_posture.joint_names = ["Gripper_Servo_Gear_Joint", "Gripper_Idol_Gear_Joint"]
	pos = JointTrajectoryPoint() 
	pos.positions.append(0.8)
	pos.positions.append(-0.8)
	pos.time_from_start = rospy.Duration(0.5)
	grasp_posture.points.append(pos)
	return grasp_posture

def createGraspPose():
	grasp.grasp_pose.header.frame_id = "base_link"
	grasp.pre_grasp_posture = preGraspPosture()
	grasp.grasp_posture = graspPosture()

	orientation = quaternion_from_euler(-1.57, 0, 0.785)

	grasp.grasp_pose.pose.position.x = -0.26
	grasp.grasp_pose.pose.position.y = 0.26
	grasp.grasp_pose.pose.position.z = 0.28
	grasp.grasp_pose.pose.orientation.x = orientation[0]
	grasp.grasp_pose.pose.orientation.y = orientation[1]
	grasp.grasp_pose.pose.orientation.z = orientation[2]
	grasp.grasp_pose.pose.orientation.w = orientation[3]

	grasp.pre_grasp_approach.direction.header.frame_id = 'base_link'
	grasp.pre_grasp_approach.direction.vector.z = -1.0	
	grasp.pre_grasp_approach.min_distance = 0.01
	grasp.pre_grasp_approach.desired_distance = 0.10

	grasp.post_grasp_retreat.direction.header.frame_id = 'base_link'
	grasp.post_grasp_retreat.direction.vector.z = 1.0
	grasp.post_grasp_retreat.min_distance = 0.01
	grasp.post_grasp_retreat.desired_distance = 0.05


def createPlacePose():
	place_location.place_pose.header.frame_id = "base_link";
	place_location.post_place_posture = preGraspPosture()

	place_orientation = quaternion_from_euler(0, 0, -1.57)

	place_location.place_pose.pose.position.x = 0.35;
	place_location.place_pose.pose.position.y = 0.35;
	place_location.place_pose.pose.position.z = 0.28;
	place_location.place_pose.pose.orientation.x = place_orientation[0]
	place_location.place_pose.pose.orientation.y = place_orientation[1]
	place_location.place_pose.pose.orientation.z = place_orientation[2]
	place_location.place_pose.pose.orientation.w = place_orientation[3]

	place_location.pre_place_approach.direction.header.frame_id = 'base_link'
	place_location.pre_place_approach.direction.vector.z = -1.0
	place_location.pre_place_approach.min_distance = 0.01
	place_location.pre_place_approach.desired_distance = 0.05


	place_location.post_place_retreat.direction.header.frame_id = 'base_link'
	place_location.post_place_retreat.direction.vector.z = 1.0

	place_location.post_place_retreat.min_distance = 0.01
	place_location.post_place_retreat.desired_distance = 0.10

def attemptPick():
	move_group.pick('part', grasp)

def attemptPlace():
	move_group.place('part', place_location)


createworld()
rospy.sleep(5)
setMoveGroupSettings()
createGraspPose()
attemptPick()
#clearOctomap()
createPlacePose()
attemptPlace()
#clearOctomap()
