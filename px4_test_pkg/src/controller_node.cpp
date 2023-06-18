#include "ros/ros.h"
#include "px4_test_pkg/controller.h"
#include "tf2_ros/buffer.h" // buffer
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"controller");

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);
    
    UAV::controller ctr("controller", &buffer);

    ros::spin();

    return 0;
}