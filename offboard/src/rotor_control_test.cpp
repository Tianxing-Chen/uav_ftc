#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <iostream>
#include "quad_msgs/ControlCommand.h"
#include <mavros_msgs/RotorControl.h>
using namespace std;


quad_msgs::ControlCommand current_state;
void state_cb(const quad_msgs::ControlCommand::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "rotor_control");
  ros::NodeHandle n;
  ros::Publisher rotor_control_pub_ = n.advertise<mavros_msgs::RotorControl>("/uav1/mavros/rotorcontrol/rotor_control", 1);
  ros::Subscriber motor_command_sub_ = n.subscribe("/hummingbird/control_command", 10, state_cb);  
  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    mavros_msgs::RotorControl rotor_control_msg_;
    for (int i=0; i<4;i++) {
      rotor_control_msg_.mot_throttle[i] = current_state.mot_throttle[i];
    } 
    rotor_control_pub_.publish(rotor_control_msg_);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
