# Build my world

Build the project:

```sh
cd ~/udacity_robotics_sw_engineer/2-gazebo_world/project/build_my_world
mkdir build && cd build
cmake .. && make
```

Run the gazebo world:
```sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/models/gazebo_models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/udacity_robotics_sw_engineer/2-gazebo_world/project/build_my_world/build
gazebo ~/udacity_robotics_sw_engineer/2-gazebo_world/project/build_my_world/world/UdacityOffice.world --verbose

```
