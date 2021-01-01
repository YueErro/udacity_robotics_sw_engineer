#!/bin/sh

xterm  -e  " source $HOME/github/udacity_robotics_sw_engineer/6-path_planning_and_navigation/project6/home_service_robot/devel/setup.bash" &
sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_gazebo turtlebot3_world.launch" &
sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_slam turtlebot3_slam.launch" &
sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
