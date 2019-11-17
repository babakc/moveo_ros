#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
from msgpack import loads
import time
from datetime import datetime 
from moveo_moveit.msg import ArmJointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import math


stepsPerRevolution = [65600,18000,4500,6560,28800]
arm_steps = ArmJointState()
total = ArmJointState()
joint_status = 0
prev_angle = [0,0,0,0,0,0] 
init_angle = [0,0,0,0,0,0]
total_steps = [0,0,0,0,0,0]

def cmd_cb(cmd_arm):
  if (cmd_arm.name[0] == "Joint_1"):
    arm_steps.position1 = (cmd_arm.position[0]-prev_angle[0])*stepsPerRevolution[0]/(2*math.pi)
    arm_steps.position2 = (cmd_arm.position[1]-prev_angle[1])*stepsPerRevolution[1]/(2*math.pi)
    arm_steps.position3 = ((cmd_arm.position[2]-prev_angle[2])*stepsPerRevolution[2]/(2*math.pi))
    arm_steps.position4 = ((cmd_arm.position[3]-prev_angle[3])*stepsPerRevolution[3]/(2*math.pi))
    arm_steps.position5 = ((cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/(2*math.pi))
	
    prev_angle[0] = cmd_arm.position[0]
    prev_angle[1] = cmd_arm.position[1]
    prev_angle[2] = cmd_arm.position[2]
    prev_angle[3] = cmd_arm.position[3]
    prev_angle[4] = cmd_arm.position[4]
		
    total.position1 += arm_steps.position1
    total.position2 += arm_steps.position2
    total.position3 += arm_steps.position3
    total.position4 += arm_steps.position4
    total.position5 += arm_steps.position5
    global joint_status
    joint_status = 1
        
  
  if (cmd_arm.name[0] == "Gripper_Idol_Gear_Joint"):
    if (cmd_arm.position[1] >= 0.5):
      total.position6 = 60
    elif (cmd_arm.position[1] < 0.5):
      total.position6 = 0
    #global joint_status
    #joint_status = 1
        


rospy.init_node('moveo_moveit')
rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, cmd_cb)  
pub = rospy.Publisher('/joint_steps', ArmJointState, queue_size=50)
#r = rospy.Rate(1000)

while not rospy.is_shutdown():
  if(joint_status == 1):
    joint_status = 0
    print ""
    print "total"
    print  total  
    print ""
    print "pubishing to /joint_steps"
    pub.publish(total)
  #rospy.spin()
  #rospy.sleep(1)