/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <moveo_moveit/ArmJointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Joint 1
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// Joint 2
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

// Joint 3
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

// Joint 4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// Joint 5 
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

AccelStepper joint1(1,E0_STEP_PIN, E0_DIR_PIN);
AccelStepper joint2(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint3(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint4(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint5(1, E1_STEP_PIN, E1_DIR_PIN);

Servo gripper;
MultiStepper steppers;

long joint_step[6];
int joint_status = 0;

ros::NodeHandle nh;
std_msgs::Int16 msg;

//instantiate publisher (for debugging purposes)
ros::Publisher steps("joint_steps_feedback",&msg);

void arm_cb(const moveo_moveit::ArmJointState& arm_steps){

  joint_step[0] = -arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = -arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6; //gripper position <0-180>
  
  gripper.write(joint_step[5]);  // move gripper after manipulator reaches goal   
  delay(350);
  steppers.moveTo(joint_step);
  steppers.runSpeedToPosition(); // Blocks until all are in position
    
  // Publish back to ros to check if everything's correct
  msg.data=joint_step[4];
  steps.publish(&msg);
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}

//instantiate subscribers
ros::Subscriber<moveo_moveit::ArmJointState> arm_sub("joint_steps", arm_cb); //subscribes to joint_steps on arm
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  //put your setup code here, to run once:
  //Serial.begin(500000);
  nh.getHardware()->setBaud(500000);
  pinMode(13,OUTPUT);
  joint_status = 1;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  nh.advertise(steps);

  // Configure each stepper
  //joint1.setMaxSpeed(300);
  //joint2.setMaxSpeed(300);
  //joint3.setMaxSpeed(600);
  //joint4.setMaxSpeed(100);
  //joint5.setMaxSpeed(100);


  joint1.setMaxSpeed(6000);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(300);
  joint4.setMaxSpeed(1000);
  joint5.setMaxSpeed(2000);

  joint1.setEnablePin(E0_ENABLE_PIN);
  joint1.setPinsInverted(false, false, true);
  joint1.enableOutputs();

  joint2.setEnablePin(Z_ENABLE_PIN);
  joint2.setPinsInverted(false, false, true);
  joint2.enableOutputs();

  joint3.setEnablePin(Y_ENABLE_PIN);
  joint3.setPinsInverted(false, false, true);
  joint3.enableOutputs();

  joint4.setEnablePin(X_ENABLE_PIN);
  joint4.setPinsInverted(false, false, true);
  joint4.enableOutputs();

  joint5.setEnablePin(E1_ENABLE_PIN);
  joint5.setPinsInverted(false, false, true);
  joint5.enableOutputs();


  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  // Configure gripper servo
  gripper.attach(11);
  gripper.write(0);  
  digitalWrite(13, 1); //toggle led
  
}

void loop() {
  nh.spinOnce();
  delay(10);  
}
