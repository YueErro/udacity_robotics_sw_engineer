# ball_chaser

Build, source and launch the gazebo world:
```sh
cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/src
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch rviz:=false
```

![](images/my_robot.png)

In a new terminal run the `drive_bot` node:
```sh
source ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
rosrun ball_chaser drive_bot
```

In a new different terminal request a `ball_chaser/command_robot` service:
```sh
source ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
# Drive your robot forward
rosservice call /ball_chaser/command_robot "linear_x: 0.5
angular_z: 0.0"
# Drive your robot left
rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.5"
# Drive your robot right
rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: -0.5"
# Bring your robot to a complete stop
rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.0"
```
