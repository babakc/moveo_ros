import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from std_srvs.srv import Empty, EmptyRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from copy import deepcopy
from random import shuffle
import copy
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name

def createPickupGoal(group="arm", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target
	pug.group_name = group
	pug.possible_grasps.extend(possible_grasps)
	pug.allowed_planning_time = 15
	pug.planning_options.planning_scene_diff.is_diff = True
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True
	pug.planning_options.plan_only = False
	pug.planning_options.replan = True
	pug.planning_options.replan_attempts = 50
	pug.allowed_touch_objects = []
	pug.attached_object_touch_links = ['<octomap>']
	pug.attached_object_touch_links.extend(links_to_allow_contact)

	return pug


def createPlaceGoal(place_pose,
                                place_locations,
				group="arm",
				target="part",
				links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group
	placeg.attached_object_name = target
	placeg.place_locations = place_locations
	placeg.allowed_planning_time = 15.0
	placeg.planning_options.planning_scene_diff.is_diff = True
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
	placeg.planning_options.plan_only = False
	placeg.planning_options.replan = True
	placeg.planning_options.replan_attempts = 50
	placeg.allowed_touch_objects = ['<octomap>']
	placeg.allowed_touch_objects.extend(links_to_allow_contact)

	return placeg


class PickAndPlaceServer(object):
	def __init__(self):
		rospy.loginfo("Initalizing PickAndPlaceServer...")
		rospy.loginfo("Connecting to pickup AS")
		self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
		self.pickup_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to place AS")
		self.place_ac = SimpleActionClient('/place', PlaceAction)
		self.place_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		self.scene = PlanningSceneInterface()
		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy(
			'/get_planning_scene', GetPlanningScene)
		self.scene_srv.wait_for_service()
		rospy.loginfo("Connected.")

		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo("Connected!")

		# Get the links of the end effector exclude from collisions
		self.links_to_allow_contact = ["Tip_Gripper_Idol", "Gripper_Idol_Gear_Joint"]
		self.pick_as = SimpleActionServer(
			'/pickup_pose', PickUpPoseAction,
			execute_cb=self.pick_cb, auto_start=False)
		self.pick_as.start()
		
	        self.place_as = SimpleActionServer(
			'/place_pose', PickUpPoseAction,
			execute_cb=self.place_cb, auto_start=False)
		self.place_as.start()

	def pick_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.pick_as.set_aborted(p_res)
		else:
			self.pick_as.set_succeeded(p_res)


	def place_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.place_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.place_as.set_aborted(p_res)
		else:
			self.place_as.set_succeeded(p_res)


	def wait_for_planning_scene_object(self, object_name='part'):
		rospy.loginfo(
			"Waiting for object '" + object_name + "' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
		
		part_in_scene = False
		while not rospy.is_shutdown() and not part_in_scene:
			# This call takes a while when rgbd sensor is set
			gps_resp = self.scene_srv.call(gps_req)
			# check if 'part' is in the answer
			for collision_obj in gps_resp.scene.world.collision_objects:			
				if collision_obj.id == object_name:
					part_in_scene = True
					break
			else:
				rospy.sleep(1.0)

		rospy.loginfo("'" + object_name + "' is in scene!")

	def grasp_object(self, object_pose):
		rospy.loginfo("Removing any previous 'part' object")
		self.scene.remove_world_object("part")
		self.scene.remove_world_object("table")
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())
		rospy.sleep(2.0)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")

		rospy.loginfo("Object pose: %s", object_pose.pose)
		
                #Add object description in scene
		self.scene.add_box("part", object_pose, (0.03, 0.03, 0.15))

		rospy.loginfo("Second %s", object_pose.pose)
		table_pose = copy.deepcopy(object_pose)

                #define a virtual table below the object
                table_height = object_pose.pose.position.z - 0.02/2 
                table_width  = 0.1
                table_depth  = 0.05
                table_pose.pose.position.z += -(0.04)/2 -0.15/2
                table_height -= 0.008 #remove few milimeters to prevent contact between the object and the table

		#self.scene.add_box("table", table_pose, (table_depth, table_width, table_height))
		
		# # We need to wait for the object part to appear
		self.wait_for_planning_scene_object()
		#self.wait_for_planning_scene_object("table")
		
		
                # compute grasps
		possible_grasps = self.create_grasp()
		self.pickup_ac
		goal = createPickupGoal(
			"arm", "part", object_pose, possible_grasps, self.links_to_allow_contact)
		
                rospy.loginfo("Sending goal")
		self.pickup_ac.send_goal(goal)
		rospy.loginfo("Waiting for result")
		self.pickup_ac.wait_for_result()
		result = self.pickup_ac.get_result()
		rospy.logdebug("Result: " + str(result))
		rospy.loginfo(
			"Pick result: " +
		str(moveit_error_dict[result.error_code.val]))
		
		return result.error_code.val


	def place_object(self, object_pose):
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())
		possible_placings = self.createPlacePose()
		# Try only with arm
		rospy.loginfo("Trying to place using only arm")
		goal = createPlaceGoal(
			object_pose, possible_placings, "arm", "part", self.links_to_allow_contact)
		rospy.loginfo("Sending goal")
		self.place_ac.send_goal(goal)
		rospy.loginfo("Waiting for result")

		self.place_ac.wait_for_result()
		result = self.place_ac.get_result()
		rospy.loginfo(str(moveit_error_dict[result.error_code.val]))

                # print result
		rospy.loginfo(
			"Result: " +
			str(moveit_error_dict[result.error_code.val]))
		rospy.loginfo("Removing previous 'part' object")
		self.scene.remove_world_object("part")

		return result.error_code.val

	def create_grasp(self):
		grasps = []
		grasp = moveit_msgs.msg.Grasp()
		grasp.grasp_pose.header.frame_id = "base_link"
		grasp.pre_grasp_posture = preGraspPosture()
		grasp.grasp_posture = graspPosture()

		#orientation = quaternion_from_euler(-1.57, 0.10, 0.00)
                orientation = quaternion_from_euler(-1.57, 0.0, 0.785)

		#grasp.grasp_pose.pose.position.x = -0.02
		#grasp.grasp_pose.pose.position.y = 0.37
		grasp.grasp_pose.pose.position.x = -0.25
		grasp.grasp_pose.pose.position.y = 0.25
		
		grasp.grasp_pose.pose.position.z = 0.38
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
		#grasp.max_contact_force = self.max_contact_force
		#grasp.allowed_touch_objects = self._allowed_touch_objects
		grasps.append(grasp)		
		return grasps

        def createPlacePose(self):
                place_locations = []
                place_location = moveit_msgs.msg.PlaceLocation()
	        place_location.place_pose.header.frame_id = "base_link";
	        place_location.post_place_posture = preGraspPosture()

	        #place_orientation = quaternion_from_euler(0, -0.10, -0.785)
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
	        place_location.pre_place_approach.desired_distance = 0.10


	        place_location.post_place_retreat.direction.header.frame_id = 'base_link'
	        place_location.post_place_retreat.direction.vector.z = 1.0

	        place_location.post_place_retreat.min_distance = 0.01
	        place_location.post_place_retreat.desired_distance = 0.05
	        place_locations.append(place_location)		
		return place_locations

def preGraspPosture(): 
	pre_grasp_posture = JointTrajectory() 
	pre_grasp_posture.header.frame_id = "base_link" 
	pre_grasp_posture.header.stamp = rospy.Time.now()
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
	grasp_posture.header.stamp = rospy.Time.now()
	grasp_posture.joint_names = ["Gripper_Servo_Gear_Joint", "Gripper_Idol_Gear_Joint"]
	pos = JointTrajectoryPoint() 
	pos.positions.append(0.75)
	pos.positions.append(-0.75)
	pos.time_from_start = rospy.Duration(0.5)
	grasp_posture.points.append(pos)
	return grasp_posture

if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
	rospy.spin()
