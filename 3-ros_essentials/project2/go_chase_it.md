# Go chase it!

## Table of contents
* [Summary of tasks](#summary-of-tasks)
* [Directory structure](#directory-structure)
* [Project specification](#project-specification)
  * [Basic requirements](#basic-requirements)
  * [Robot design](#robot-design)
  * [Gazebo world](#gazebo-world)
  * [Ball chasing](#ball-chasing)
  * [Launch files](#launch-files)

### Summary of tasks

In this project, you should create two ROS packages inside your `catkin_ws/src`: the `drive_bot` and the `ball_chaser`. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:
1. `drive_bot`:
  * Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
  * Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot's differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
  * House your robot inside the world you built in the **Build My World** project.
  * Add a white-colored ball to your Gazebo world and save a new copy of this world.
  * The `world.launch` file should launch your world with the white-colored ball and your robot.
2. `ball_chaser`:
  * Create a `ball_chaser` ROS package to hold your C++ nodes.
  * Write a `drive_bot`C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
  * Write a `process_image` C++ node that reads your robot's camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
  * The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.

The robot you design in this project will be used as a base model for all your upcoming projects in this Robotics Software Engineer Nanodegree Program.

### Directory structure
```sh
.Project2                          # Go Chase It Project
├── my_robot                       # my_robot package
│   ├── launch                     # launch folder for launch files
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── world                      # world folder for world files
│   │   ├── <yourworld>.world
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package
│   ├── launch                     # launch folder for launch files
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_images.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
└──
```

### Project specification

#### Basic requirements
Does the submission include the **my_robot** and the **ball_chaser** ROS packages?

Do these packages follow the directory structure detailed in the project description section?

#### Robot design
Does the submission include a design for a differential drive robot, using the Unified Robot Description Format?

Robot design requirements:

Structure basic requirements:
* Lidar and camera sensors.
* Gazebo plugins for the robot's differential drive, lidar, and camera.
* Housed inside the world.
* Significant changes from the sample taught in the project lesson.
* Robot is stable when moving.

#### Gazebo world
Does the **my_robot** ROS package contain the Gazebo world?

Gazebo world requirements:

* Same as the world designed in the **Build My World** project or a new world that you design on the building editor for this project.
* Includes a white-colored ball.

#### Ball chasing
Does the **ball_chaser** ROS package contain two C++ ROS nodes to interact with the robot and make it chase a white-colored ball?

`drive_bot` requirements:
* A `ball_chaser/command_robot` service.
* Service accepts linear x and angular z velocities.
* Service publishes to the the wheel joints.
* Service returns the requested velocities.

`process_image` requirements:
* Subscribes to the robot's camera image.
* A function to analyze the image and determine the presence and position of a white ball.
* Requests a service to drive the robot towards a white ball (when present).

#### Launch files
Does the submission include `world.launch` and `ball_chaser.launch` files that launch all the nodes in this project?

`world.launch` requirements:
* Launch the world (which includes a white ball).
* Launch the robot.

`ball_chaser.launch` requirements:
* Run the `drive_bot` C++ node.
* Run the `process_image` C++ node.
