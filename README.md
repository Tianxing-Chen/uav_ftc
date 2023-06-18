# uav_ftc
## A ROS+PX4+Gezebo simulation of fault-tolerant control for a quadrotor UAV with a total rotor failure.

# 1、Environment required
1）ROS melodic.

2）Configure the PX4 Gazebo simulation required environment.

check the following environment variables

    gedit ~/.bashrc
    source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
    export ROS_PACKAGE_PATH=~/PX4-Autopilot:$ROS_PACKAGE_PATH
    export ROS_PACKAGE_PATH=~/PX4-Autopilot/Tools/sitl_gazebo:$ROS_PACKAGE_PATH

# 2、Quick Start

Download and compile the project

    cd ${YOUR_WORKSPACE_PATH}/src
    git clone git@github.com:Tianxing-Chen/uav_ftc.git
    cd ../ 
    catkin build

check the following environment variable

    gedit ~/.bashrc
    source ~/${YOUR_WORKSPACE_PATH}/devel/setup.bash

# 3、Examples

<!-- ## Example 1: fault-tolerant control for a quadrotor UAV with a total rotor failure.

Create a new terminal and run, please using the  setting in 

    roslaunch ftc_ctrl single_uav_mavros_sitl.launch

check the parameter MC_FTC_MODE = 1 of uav0 in QGC

Create a new terminal and run

    roslaunch ftc_ctrl setup_sim.launch

Create a new terminal and run

    roscd ftc_ctrl && cd scripts

Run at normal model

    ./start_rotors.sh uav0

Run at failure model

    ./stop_rotor.sh uav0 -->

## Example 1: formation control for multiple quadrotor UAVs.

Create a new terminal and run, please using the  setting in 

    roslaunch ftc_ctrl multi_uav_mavros_sitl.launch

check the following parameters in QGC

    uav0, MC_FTC_MODE = 0;
    uav1, MC_FTC_MODE = 1;
    uav2, MC_FTC_MODE = 1;
    uav3, MC_FTC_MODE = 1;
    uav4, MC_FTC_MODE = 1;
    uav5, MC_FTC_MODE = 1;
    uav6, MC_FTC_MODE = 1;

Create a new terminal and run

    roslaunch ftc_ctrl uavs_sim.launch

Run the chap05.slx (uav_ftc/simulink code/code) in simulink, and wait for the UAV0 taking off

Create a new terminal and run

    roscd ftc_ctrl && cd scripts
    ./start_uavs.sh 



# 注意事项

在Gazebo运行PX4的情况下，打开QGC，确认MC_FTC_MODE参数是否为1

