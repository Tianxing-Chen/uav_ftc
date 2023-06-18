#!/bin/bash

# rostopic pub -1 uav0/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 0.0 1.0 uav0

rostopic pub -1 uav6/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 -3.0 1.0 uav6

rostopic pub -1 uav5/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 -2.0 1.0 uav5

rostopic pub -1 uav4/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 -1.0 1.0 uav4

rostopic pub -1 uav3/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 3.0 1.0 uav3

rostopic pub -1 uav2/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 2.0 1.0 uav2

rostopic pub -1 uav1/start_rotors std_msgs/Empty --

# ./reference.sh 0.0 1.0 1.0 uav1

rostopic pub -1 start_formation std_msgs/Empty --

rostopic pub -1 start_tracking std_msgs/Empty --

#sleep 20s


