#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
// #include "quad_msgs/QuadStateEstimate.h"

#include <iostream>
#include <stdio.h>
#include <string>
using namespace std;

mavros_msgs::State state1;
void state_cb1(const mavros_msgs::State::ConstPtr &msg) { state1 = *msg; }

geometry_msgs::PoseStamped pose1;
void pose_cb1(const geometry_msgs::PoseStamped::ConstPtr &msg) { pose1 = *msg; }

geometry_msgs::PoseStamped vrpn_pose1;
void vrpn_pose_cb1(const geometry_msgs::PoseStamped::ConstPtr &msg) { vrpn_pose1 = *msg; }

// quad_msgs::QuadStateEstimate stateUpdate;
// void stateUpdate(const quad_msgs::QuadStateEstimate::ConstPtr& msg) { stateUpdate = *msg; }


int main(int argc, char **argv) {
       ros::init(argc, argv, "ground_station");
       ros::NodeHandle nh;
       // uav1
       ros::Subscriber state_sub1 = nh.subscribe<mavros_msgs::State>("uav1/mavros/state", 20, state_cb1);
       ros::Subscriber pose_sub1 = nh.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 20, pose_cb1);
       ros::Subscriber vrpn_pose_sub1 = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/uav1/pose", 20, vrpn_pose_cb1);  

       // ros::Subscriber state_est_sub_ = nh.subscribe<quad_msgs::QuadStateEstimate>("/hummingbird/state_est", 20, stateUpdate);  

       
       ros::Rate loop_rate(10);
       while (ros::ok()) {
              cout << "                 ==================== UAV1 =======================" << endl;
              printf("connected: %d, armed: %d, guided: %d,",
                     state1.connected,state1.armed, state1.guided);
              cout << "mode: " << state1.mode << endl;
              printf("position_x: %f, position_y: %f, position_z: %f\n",
                     pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z);
              printf("vrpn_position_x:%f, vrpn_position_y:%f, vrpn_position_z:%f\n",
                     vrpn_pose1.pose.position.x, vrpn_pose1.pose.position.y, vrpn_pose1.pose.position.z);     
              //  printf("stateUpdate_x:%f, stateUpdate_y:%f, stateUpdate_z:%f\n",
              //        stateUpdate.position.x, stateUpdate.position.y, stateUpdate.position.z);                 
              cout << "================================================================================\n" << endl;

              ros::spinOnce();
              loop_rate.sleep();
       }
       return 0;
}