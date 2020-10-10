# Build my world

## Table of contents
* [Summary of tasks](#summary-of-tasks)
* [Directory structure](#directory-structure)
* [Project specification](#project-specification)
  * [Basic requirements](#basic-requirements)
  * [Building](#building)
  * [Modeling](#modeling)
  * [Gazebo world](#gazebo-world)
  * [World plugin](#world-plugin)

### Summary of tasks

1. Build a single floor wall structure using the Building Editor tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.
2. Model any object of your choice using the Model Editor tool in Gazebo. Your model links should be connected with joints.
3. Import your structure and two instances of your model inside an empty Gazebo World.
4. Import at least one model from the Gazebo online library and implement it in your existing Gazebo world.
5. Write a C++ World Plugin to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

### Directory structure
```sh
.Project1                          # Build My World Project
├── model                          # Model files
│   ├── Building
│   │   ├── model.config
│   │   ├── model.sdf
│   ├── HumanoidRobot
│   │   ├── model.config
│   │   ├── model.sdf
├── script                         # Gazebo World plugin C++ script
│   ├── welcome_message.cpp
├── world                          # Gazebo main World containing models
│   ├── UdacityOffice.world
├── CMakeLists.txt                 # Link libraries
└──
```

### Project specification

#### Basic requirements
Does the project include a **world** directory containing the Gazebo world file, a **model** directory containing a structure and an object model files, a **script** directory containing the C++ plugin code, and a **CMakeLists.txt** file?

#### Building
Does the project include a house with walls?

The student designed a structure and stored it in the model directory.

Structure basic requirements:
* Structure is different than the one shown in the sample simulation world.
* Single floor.
* Enough space for robots to navigate.
* At least one feature.
* At least one color.

#### Modeling
Does the project include an object built using the **Model Editor**?

The student designed an object and stored it in the model directory.

Model basic requirements:

* Object is different than the one shown in the sample simulation world.
* Object links are connected through joints.

#### Gazebo world
Do the project contain a **Gazebo world** with multiple models?

The student created a Gazebo world and stored it in the world directory.

Gazebo World basic requirements:

* World is different than the one shown in the sample simulation world.
* Contains the structure model.
* Contains two instances of the object model.
* Contains one model from the Gazebo online library.

#### World plugin
Does the project contain a C++ **world plugin**?

The student created a C++ plugin and stored it in the script directory. Also, the student created a CMakeLists.txt file and stored in the main project directory.

World plugin basic requirements:

* The plugin C++ code should print “Welcome to <your name>’s World!” message.
* Do not submit the build directory!
