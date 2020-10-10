# Go chase it!

Build the project:

```sh
cd ~/go_chase_it
source /opt/ros/kinetic/setup.bash
catkin_make
```

Launch the gazebo world:
```sh
# Terminal 1
source devel/setup.bash
roslaunch my_robot world.launch
```

Once gazebo world has been launched, launch the nodes:
```sh
# Terminal 2
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

Move the white ball to the view of the robot's camera to chase it.
