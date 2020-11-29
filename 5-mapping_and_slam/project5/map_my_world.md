# Map my world!

## Table of contents
* [Project specification](#project-specification)
  * [Basic requirements](#basic-requirements)
  * [Simulation Setup](#simulation-setup)
  * [Mapping Package](#mapping-package)
  * [Mapping Accuracy](#mapping-accuracy)
* [Suggestions to Make Your Project Stand Out!](#suggestions-to-make-your-project-stand-out)

### Project specification
#### Basic requirements
* Did the student submit all required files?

  Student submited all required files:
  * ROS Package: robot and RTABMAP
  * Db file generated (could be link to file if oversized)

#### Simulation Setup
* Did the student set up the simulation environment properly?
  * Student's simulation world and robot could properly load in Gazebo.
* Is the student's simulation suitable for mapping task?
  * The student's environment should have clear features and geometric shapes to perform mapping.

#### Mapping Package
* Does the student correctly build all required launch files for RTAB-Mapping?

  Student created the following launch files properly:
  * `mapping.launch`
  * `teleop.launch`
  * `localization.launch`

  The student's program should be able to launch without errors

#### Mapping Accuracy
* Was the student able to generate a 3D map using RTAB-Map?
  * Student's map should contain at least 3 loop closures and the occupancy grid is identifiable
* Does the student's 3D map portray environment characteristics?
  * Student's map should clearly portray the environment. The student should be able to display the characteristics of the landmark features.
