#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <moveo_moveit/ArmJointArray.h>
#include <moveo_moveit/ArmJointState.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

ros::NodeHandle nh;


std_msgs::Int16 mydata;
ros::Publisher chatter("chatter", &mydata);

void cmd_cb(const moveo_moveit::ArmJointArray& cmd_arm)
{
  
   for( unsigned int a = 0; a <= cmd_arm.ArmJoints_length; a = a + 1 )
   {
    mydata.data = cmd_arm.ArmJoints[a].position5;
    chatter.publish( &mydata );
   }
 }

 
ros::Subscriber<moveo_moveit::ArmJointArray> sub("/joint_steps", cmd_cb);

void setup()
{
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop()
{

  nh.spinOnce();
}
