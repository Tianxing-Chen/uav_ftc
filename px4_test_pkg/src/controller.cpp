#include "px4_test_pkg/controller.h"

namespace UAV
{

controller::controller(){}
controller::~controller(){}

controller::controller(const std::string name, tf2_ros::Buffer *tf):
    mission_(IDLE)
{
    
    static ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("uav_name", uav_name_, std::string("uav1"));    

    uav_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ uav_name_+"/pose", 10, &controller::uav_pose_cb, this);
    // uav_state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &controller::uav_state_cb, this);//buchong
    vision_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    goal_pub_ = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",10);

    mission_server_ = nh.advertiseService("mission", &controller::mission_cb, this);

    tf2::toMsg(tf2::Transform::getIdentity(), uav_pose_.pose);

    goal_.type_mask = 0b100111111000;
    goal_.coordinate_frame = 1;
    goal_.position.x = 0;
    goal_.position.y = 0;
    goal_.position.z = 1.0;
    goal_.yaw = 0;

    vision_timer_ = nh.createTimer(ros::Duration(0.01), &controller::vision_timer_cb, this);
    mainloop_timer_ = nh.createTimer(ros::Duration(0.05), &controller::mainloop_timer_cb, this);

}

void controller::mainloop_timer_cb(const ros::TimerEvent &event){
    switch(mission_){
        case IDLE:
            goal_.position.x = uav_pose_.pose.position.x;
            goal_.position.y = uav_pose_.pose.position.y;
            goal_.position.z = uav_pose_.pose.position.z;
            break;

        case TAKEOFF:
            goal_.position.x = last_x;
            goal_.position.y = last_y;
            goal_.position.z = 1.0;
            break;

        case MOVE:
            goal_.position.z = 1.0;
            break;

        case LANDING:
            goal_.position.x = last_x;
            goal_.position.y = last_y;
            goal_.position.z = std::max(last_z, uav_pose_.pose.position.z - 0.2);
            break;

        default:
            ROS_ERROR("%s: unvalid mission.", uav_name_.c_str());
    }
    goal_pub_.publish(goal_);
}

bool controller::mission_cb(px4_test_pkg::Mission::Request& req, px4_test_pkg::Mission::Response& resp){
    switch(req.mission){
        case 1: // takeoff
            if (mission_ == IDLE || mission_ == TAKEOFF){
                mission_ = TAKEOFF;
                last_x = uav_pose_.pose.position.x;
                last_y = uav_pose_.pose.position.y;
                last_z = uav_pose_.pose.position.z;
                resp.success = true;
                resp.message = uav_name_ + ": taking off.";
            }
            else{
                resp.success = false;
                resp.message = uav_name_ + ": can't take off.";
            }
            break;
        case 2: // move
            if (mission_ == TAKEOFF || mission_ == MOVE){
                mission_ = MOVE;

                goal_.position.x = req.x;
                goal_.position.y = req.y;

                resp.success = true;
                std::stringstream ss;
                ss << uav_name_ << ": moving to (" << req.x << ", " << req.y << ").";
                resp.message = ss.str();
            }
            else{
                resp.success = false;
                resp.message = uav_name_ + ": can't move.";
            }
            break;
        case 3: // land
            if (mission_ == TAKEOFF || mission_ == MOVE){
                mission_ = LANDING;
                last_x = uav_pose_.pose.position.x;
                last_y = uav_pose_.pose.position.y;
                resp.success = true;
                resp.message = uav_name_ + ": landing.";
            }
            else{
                resp.success = false;
                resp.message = uav_name_ + ": landing fail.";
            }
            break;
        default:
            resp.success = false;
            resp.message = uav_name_ + ": unvalid mission!";
    }
}

}