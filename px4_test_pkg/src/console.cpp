#include "ros/ros.h"
#include "px4_test_pkg/Mission.h"
#include <string>

// global variables
std::string uav_name;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"uav_console");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("uav_name", uav_name, std::string("uav1"));    

    ros::ServiceClient client = nh.serviceClient<px4_test_pkg::Mission>(uav_name + "/mission");
    ros::service::waitForService(uav_name + "/mission");
    
    px4_test_pkg::Mission m;

    int n;
    bool flag;

    while(ros::ok()){
        std::cout << "mission: 1) TAKEOFF 2) MOVE 3) LAND" << std::endl;
        std::cin >> n;
        
        if (n==1 || n==2 || n==3){
            if(n==2){
                std::cout << "x y:" << std::endl;
                std::cin >> m.request.x;
                std::cin >> m.request.y;
            }
            m.request.mission = n;
            flag = client.call(m);
            if (flag)
            {
                ROS_INFO("%s", m.response.message.c_str());
            }
            else
            {
                ROS_ERROR("请求处理失败....");
                return 1;
            }
        }
        else{
            ROS_ERROR("error input!");
        }

    }

    return 0;
}