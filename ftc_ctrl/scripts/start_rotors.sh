#!/bin/bash

rostopic pub -1 $1/start_rotors std_msgs/Empty --

./reference.sh 0.0 1.5 1.0 $1


#sleep 20s


