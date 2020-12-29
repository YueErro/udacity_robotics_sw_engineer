# Home Service Robot

## Table of contents
- [Demo](#demo)
- [Brief packages write-up](#brief-packages-write-up)
- [Build the project](#build-the-project)
- [Run SLAM](#run-slam)
  - [Automatically](#automatically)
  - [Manually](#manually)
- [Run navigation](#run-navigation)
  - [Automatically](#automatically-1)
  - [Manually](#manually-1)
- [RUN add markers node](#run-add-markers-node)
  - [Automatically](#automatically-2)
  - [Manually](#manually-2)
- [RUN pick objects](#run-pick-objects)
  - [Automatically](#automatically-3)
  - [Manually](#manually-3)
- [Run home service](#run-home-service)
  - [Automatically](#automatically-4)
  - [Manually](#manually-4)

### Demo

### Brief packages write-up
This project make use of the following packages:
- `map`: Inside this directory, there are stored the gazebo world file and the map generated from SLAM.
- `scripts`: Inside this directory, there are the shell scripts done during this project.
- `rvizConfig`: Inside this directory, there is the customized RViz configuration file for this project.
- `add_markers`: This is a ROS package containing a node that models the object with a marker in RViz.
- `pick_objects`: This is a ROS package containing a node that commands the robot to drive to the pickup and drop off zones.
- `slam_gmapping`: It contains a wrapper around gmapping which provides SLAM capabilities. `gmapping` is a ROS package that provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping.
- `turtlebot3`: It contains ROS packages for the Turtlebot3 (meta package).
- `turtlebot3_simulations`: It contains ROS packages for the Turtlebot3 simulation (meta package).

Using `slam-gmapping` a 2-D occupancy grid map is created from laser and pose data collected by the Turtlebot3 robot (`turtlebot3` ROS package) driven by the user using the teleop keyboard from `turtlebot3_teleop` ROS package.

From the map created, the AMCL (Adaptative Monte Carlo Localization) ROS node is used from `turtlebot3_navigation` ROS package in order to localize the robot within the environment using a particle filter to track the pose of the robot against the known map.

Everything is displayed on RViz, as usual, in this case with a customized configuration.

In order to simulate a pickup and drop off movement, the ROS packages `add_markers` and `pick_objects` have been communicated with each other. In other words, the robot moves according to the goal set by the `pick_objects` node, and the `add_markers` node adds a marker in the pickup zone and subscribes to the odometry to keep track of the Turtlebot3's pose. In such a way that once the robot reaches the pickup zone, it deletes the marker, and afterwards, once the robot reaches the drop off zone, it adds the marker in that position again.

### Build the project
```sh
cd ~/home_service_robot
source /opt/ros/melodic/setup.bash
catkin_make
```

### Run SLAM
#### Automatically
```sh
bash ~/home_service_robot/src/scripts/test_slam.sh
```

#### Manually
Launch the world:
```sh
source ~/home_service_robot/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch SLAM:
```sh
source ~/home_service_robot/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Launch `teleop` to control the robot:
```sh
source ~/home_service_robot/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Run navigation
#### Automatically
```sh
bash ~/home_service_robot/src/scripts/test_navigation.sh
```

#### Manually
Launch the world:
```sh
source ~/home_service_robot/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch navigation:
```sh
source ~/home_service_robot/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

### RUN add markers node
#### Automatically
```sh
bash ~/home_service_robot/src/scripts/add_marker.sh
```

#### Manually
Launch the world and the navigation explained above: [manually run navigation](#manually-1)

Run add markers node:
```sh
source ~/home_service_robot/devel/setup.bash
rosrun add_markers add_markers_node
```

### RUN pick objects
#### Automatically
```sh
bash ~/home_service_robot/src/scripts/pick_objects.sh
```

#### Manually
Launch the world and the navigation explained above: [manually run navigation](#manually-1)

Run pick objects node:
```sh
source ~/home_service_robot/devel/setup.bash
rosrun pick_objects pick_objects_node
```

### Run home service
#### Automatically
```sh
bash ~/home_service_robot/src/scripts/home_service.sh
```

#### Manually
Launch the world and the navigation explained above: [manually run navigation](#manually-1).

Run add markers node explained above: [manually run add markers node](#manually-2).

Run pick objects node explained above: [manually run pick objects node](#manually-3).
