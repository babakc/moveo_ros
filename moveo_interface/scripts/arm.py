#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
import sys
import copy
from msgpack import loads
import time
from datetime import datetime 

from moveo_interface.msg import ArmJointState, ArmJointArray

import numpy as np
import actionlib
import math
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionResult, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryResult
from control_msgs.msg import GripperCommandAction, GripperCommandActionResult, GripperCommandFeedback
from actionlib_msgs.msg import GoalStatus

lJointNames = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Gripper_Servo_Gear_Joint', 'Gripper_Idol_Gear_Joint', 'Pivot_Arm_Gripper_Servo_Joint', 'Pivot_Arm_Gripper_Idol_Joint']

class arm_control(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryActionFeedback()
    _result = FollowJointTrajectoryActionResult()
    _gripper_feedback = GripperCommandFeedback()
    _gripper_result = GripperCommandActionResult()
    total = ArmJointState()

    def __init__(self, name):
        self.pub = rospy.Publisher('/joint_steps', ArmJointArray, queue_size=50)
        self._as_arm = actionlib.SimpleActionServer("moveo_urdf/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_joint_trajectory, auto_start = False)
        self._as_arm.start()
        self._as_gripper = actionlib.SimpleActionServer("moveo_urdf/hand_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_gripper_action, auto_start = False)
        self._as_gripper.start()
        self.pub_joint_states = rospy.Publisher("/moveo_urdf/joint_states", JointState, queue_size=500)
        self.lJointAngles = [0, 0, 0, 0, 0, 0, 0, 0]
        self.count = 0
        self.run()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish_joint_states()
            rate.sleep()
			
    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = lJointNames
        # position of last motor (gripper) needs to be adjusted by 0.35, also value is expected two times: For right and left gripper joint.
        self.lAngles = self.lJointAngles
        joint_state.position = self.lAngles[:-1] + [self.lAngles[-1], self.lAngles[-1]]
        self.pub_joint_states.publish(joint_state)

    def execute_joint_trajectory(self, goal):
        stepsPerRevolution = [65600,18000,4500,6560,28800]
        arm_steps = ArmJointState()
        arm_array = ArmJointArray()
        temp_total = ArmJointState()
        count = 0
        self._result.status = FollowJointTrajectoryResult.SUCCESSFUL
        for point in goal.trajectory.points:
            print goal.trajectory.joint_names
            print point.positions
            # get new desired joint values in order from link1_joint to link5_joint
            lGoalPosOrdered = [
                point.positions[goal.trajectory.joint_names.index(lJointNames[0])],
                point.positions[goal.trajectory.joint_names.index(lJointNames[1])],
                point.positions[goal.trajectory.joint_names.index(lJointNames[2])],
                point.positions[goal.trajectory.joint_names.index(lJointNames[3])],
                point.positions[goal.trajectory.joint_names.index(lJointNames[4])]
            ]
            try:
                print "set the joints on the hardware"
                arm_steps.position1 = (lGoalPosOrdered[0]-self.lJointAngles[0])*stepsPerRevolution[0]/(2*math.pi)
                arm_steps.position2 = (lGoalPosOrdered[1]-self.lJointAngles[1])*stepsPerRevolution[1]/(2*math.pi)
                arm_steps.position3 = ((lGoalPosOrdered[2]-self.lJointAngles[2])*stepsPerRevolution[2]/(2*math.pi))
                arm_steps.position4 = ((lGoalPosOrdered[3]-self.lJointAngles[3])*stepsPerRevolution[3]/(2*math.pi))
                arm_steps.position5 = ((lGoalPosOrdered[4]-self.lJointAngles[4])*stepsPerRevolution[4]/(2*math.pi))

                self.total.position1 += arm_steps.position1
                self.total.position2 += arm_steps.position2
                self.total.position3 += arm_steps.position3
                self.total.position4 += arm_steps.position4
                self.total.position5 += arm_steps.position5

                temp_total = copy.copy(self.total)

                arm_array.ArmJoints.append(temp_total)
                self.lJointAngles = [ lGoalPosOrdered[0], lGoalPosOrdered[1], lGoalPosOrdered[2], lGoalPosOrdered[3], lGoalPosOrdered[4], 0, 0, 0 ]
                print self.total.position5
                if count % 3 == 0:
                    print "What?"
                    self.pub.publish(arm_array)
                    time.sleep(1)
                count = count + 1
                
            except Exception, e: 
                print str(e)
                # reject if position is impossible for the hardware to reach
                self._feedback.status = GoalStatus.REJECTED
                self._as_arm.publish_feedback(self._feedback.feedback)
                self._result.status = FollowJointTrajectoryResult.INVALID_GOAL
                break

	        # publish current position error while driving to position
            error = 0
            while True:
                error = [0.0, 0.0, 0.0]
                print "Error", error
                # Position reached, done
                if all(abs(f) < 0.15 for f in error): # allow a tiny tolerance and continue with next goal
                    print "Position reached"
                    self._feedback.status = GoalStatus.SUCCEEDED
                    break
                print "stuck in beast mode????"
                # Cancel if requested
                if self._as_arm.is_preempt_requested():
                    self._feedback.status = GoalStatus.PREEMPTING
                    self._as_arm.set_preempted()
                    break
                sleep(0.001)
                
			# Give feedback to either canceled or current position reached
            self._feedback.feedback.joint_names = lJointNames[:-1]
            self._feedback.feedback.desired.positions = lGoalPosOrdered
            self._feedback.feedback.actual.positions = self.lAngles[:-1]
            self._feedback.feedback.error.positions = error
            self._as_arm.publish_feedback(self._feedback.feedback)
        
        print arm_array
        self._as_arm.set_succeeded(self._result.result)

    def execute_gripper_action(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)
          
        if success:
            self._result.sequence = self._feedback.sequence
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        

if __name__ == '__main__':
    rospy.init_node('arm_control')
    server = arm_control(rospy.get_name())
    rospy.spin()