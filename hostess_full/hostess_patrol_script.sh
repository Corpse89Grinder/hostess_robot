#!/bin/bash
#-*- ENCODING: UTF-8 -*-

#echo "setting ROS configuration..."
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.0.175
echo "updating time"
sudo ntpdate -s it.pool.ntp.org
echo "rebooting r2p"
./pbmanager.py -d /dev/pbnuzoo -e "load 2 off" 
sleep 0.5
./pbmanager.py -d /dev/pbnuzoo -e "load 2 on"
echo "config WS"
source /home/rob/hostess_ws/devel/setup.bash
echo "launching roscore"
roscore &
sleep 5
echo "roscore ready"
echo "roslaunch config"
roslaunch hostess_full hostess_robot.launch &
roslaunch rosserial_r2p rosserial_server.launch &
roslaunch hokuyo_node hokuyo_test.launch &
roslaunch kobra_odom kobra_odom.launch &
roslaunch npb npd.launch &
roslaunch teleop_asist teleop_asist_hostess.launch &
roslaunch robot_pose_publisher robot_pose_publisher.launch &
roslaunch ompl_planner_rrt virgil_patrol.launch & 
echo "roslaunch ready"
exit
