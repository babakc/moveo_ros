#!/usr/bin/python

import rospy
import os
from std_srvs.srv import Empty, EmptyRequest

os.system("rosrun gazebo_ros spawn_model -database coke_can_slim -gazebo -model grasp_cube -x -0.35 -y 0.35 -z 1")

rospy.loginfo("Connecting to clear octomap service...")
clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
clear_octomap_srv.wait_for_service()
rospy.loginfo("Connected!")
clear_octomap_srv.call(EmptyRequest())
rospy.sleep(1)
clear_octomap_srv.call(EmptyRequest())
