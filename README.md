# fault_tolerant_control
## 本工程代码是单旋翼失效的ROS+PX4+Gezebo仿真，实验部分代码待补充。

# 1、需要环境
1）ROS melodic

2）请根据Pixhark官网配置所需要环境

3）请源码安装mavros，mavlink,然后删除mavros、mavlink文件夹。此步骤的目的是安装mavros、mavlink的一些依赖，给出的源码中有修改过的mavros、mavlink文件夹，因此安装完成后需要删除这两个文件夹

# 2、下载源码编译
git clone git@github.com:Tianxing-Chen/fault_tolerant_control.git

cd fault_tolerant_control/

catkin build

打开fault_tolerant_control/文件夹，可以发现PX4-Autopilot.zip文件，剪切到home目录下，删除Pixhark官网配置的源码，并解压PX4-Autopilot.zip（此步骤的目的是PX4-Autopilot.zip更改了PX4的源码，直接更改源码的步骤有点复杂）

gedit ~/.bashrc

添加环境变量

source ~/fault_tolerant_control/devel/setup.bash

source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default

export ROS_PACKAGE_PATH=~/PX4-Autopilot:$ROS_PACKAGE_PATH

export ROS_PACKAGE_PATH=~/PX4-Autopilot/Tools/sitl_gazebo:$ROS_PACKAGE_PATH

# 3、运行代码

roslaunch px4 multi_uav_mavros_sitl.launch

roslaunch ftc_ctrl testgazebo.launch 

roscd ftc_ctrl && cd scripts

正常起飞无人机

./start_rotors.sh hummingbird

单翼失效下飞行

./stop_rotor.sh hummingbird

# 注意事项
在Gazebo运行PX4的情况下，打开QGC，确认MC_FTC_MODE参数是否为1

