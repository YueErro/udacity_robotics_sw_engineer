# Map my world!

Build the project:
```sh
cd ~/map_my_world
source /opt/ros/kinetic/setup.bash
catkin_make -DRTABMAP_SYNC_MULTI_RGBD=ON
```

Launch your world:
```sh
# Terminal 1
source devel/setup.bash
roslaunch my_robot world.launch
```

Launch mapping:
```sh
# Terminal 2
source devel/setup.bash
roslaunch my_robot mapping.launch
```

Use `teleop` to control the robot:
```sh
# Terminal 3
source devel/setup.bash
roslaunch my_robot teleop.launch
```

Once you are than with the mapping, stop running the mapping and check the result:
```sh
rtabmap-databaseViewer /root/.ros/rtabmap.db
```

You can check the one i have generated:
```sh
rtabmap-databaseViewer ~/map_my_world/src/rtabmap.db
```
