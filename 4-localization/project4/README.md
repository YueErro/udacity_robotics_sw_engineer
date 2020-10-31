# Where am I?

Build the project:
```sh
cd ~/where_am_i
source /opt/ros/kinetic/setup.bash
catkin_make
```

Launch your world:
```sh
# Terminal 1
source devel/setup.bash
roslaunch my_robot world.launch
```

Launch amcl:
```sh
# Terminal 2
source devel/setup.bash
roslaunch my_robo amcl.launch
```

Wait amcl to load and run RViz with amcl configuration:
```sh
# Terminal 3
source devel/setup.bash
rosrun rviz rviz -d `rospack find my_robot`/rviz/amcl.rviz
```

Use `teleop` to control the robot:
```sh
# Terminal 4
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
