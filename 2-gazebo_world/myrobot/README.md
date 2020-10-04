# myrobot

Build the project:

```sh
cd ~/udacity_robotics_sw_engineer/2-gazebo_world/myrobot
mkdir build && cd build
cmake .. && make
```

Run the gazebo world:
```sh
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/udacity_robotics_sw_engineer/2-gazebo_world/myrobot/build
gazebo ~/udacity_robotics_sw_engineer/2-gazebo_world/myrobot/world/myworld --verbose
```
