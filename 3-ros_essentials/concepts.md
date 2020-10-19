# ROS

## Table of contents
* [What is ROS?](#what-is-ROS)
  * [Components and features](#components-and-features)
  * [Nodes and Topics](#nodes-and-topics)
    * [Quiz: Nodes and Topics](#quiz-nodes-and-topics)
  * [Messages](#messages)
    * [Quiz: Messages](#quiz-messages)
  * [Publishers](#publishers)
    * [Quiz: Publisher](#quiz-publisher)
  * [Subscribers](#subscribers)
    * [Quiz: Subscriber](#quiz-subscriber)
  * [Services](#services)
    * [ServiceServer](#serviceserver)
    * [ServiceClient](#serviceclient)
    * [Quiz: Services](#quiz-services)
  * [Compute graph](#compute-graph)
  * [Environment](#environment)
    * [Quiz: Setting up the ROS environment](#quiz-setting-up-the-ros-environment)
* [Turtlesim](#turtlesim)
  * [Overview](#overview)
    * [Quiz: Commands](#quiz-commands)
  * [Run](#run)
  * [List nodes](#list-nodes)
  * [List topics](#list-topics)
  * [Get topic info](#get-topic-info)
  * [Message information](#message-information)
  * [Echo a topic](#echo-a-topic)
* [Catkin](#catkin)
  * [Workspaces](#workspaces)
  * [Packages](#packages)
  * [Create a catkin workspace](#create-a-catkin-workspace)
  * [Add a package](#add-a-package)
  * [Roslaunch](#roslaunch)
    * [Example: arm_mover](#example-arm_mover)
    * [Example: look_away](#example-look_away)
  * [Rosdep](#rosdep)
  * [Dive deeper into packages](#dive-deeper-into-packages)
* [URDF](#urdf)
* [Sensors](#sensors)
  * [Camera](#camera)
  * [Lidar](#lidar)
* [RViz](#rviz)

### What is ROS
ROS or Robot Operating System is a open-source software framework for robotics development.

#### Components and features
ROS porvides a means of communicating with hardware and a way for different processes to communicate with one another via message passing.

ROS features a slick build and package management system called **catkin**, allowing you to develop and deploy software with ease (See more [here](#catkin)).

To sum up ROS components and features:
* Open-source
* Hardware abstraction of device drivers
* Communication via message passing
* Slick build and package management
* Tools for visualization, simulation and analysis
* Powerful software libraries

#### Nodes and Topics
**ROS master** maintains the registry of all the active nodes on a system. Each node can use this registry to discover other nodes and establish lines of communication. In addition, it also host what's called the parameter server.

The **parameter server** is typically used to store parameters and configuration values, that are share among the the running nodes.Nodes can also share data with one another by passing messages over what are called topics.

**Topic** is a pipe between nodes, trough which messages flow. In order to send a message on a topic, we say that the node must publish (**publisher**) to that topic. Likewise, to receive a message on a topic, a node must subscribe (**subscriber**) to that topic.

A network of nodes connected by topics is called a publish subscribe or pops up  architecture.

##### Quiz: Nodes and Topics
The following statements are true about ROS nodes and topics:
* Robots may be very different in form and function, but they all perform the same high-level tasks of perception, decision making, and actuation.
* The parameter server acts as a central repository where nodes on a system can look up parameter values.
* Nodes pass messages to one another via topics, which you can think of as a pipe connecting two nodes.
* A single node may simultaneously publish and subscribe to many topics.

#### Messages
ROS has a wide variety of predefined message types:
* Physical quantities:
  * Position
  * Velocities
  * Accelerations
  * Rotations
  * Durations
* Sensor readings
  * Laser scans
  * Images
  * Point clouds
  * Inertial
  * Measurements

You can also define your own message that can contain any kind of data not just text.

##### Quiz: Messages
The following statements are true about ROS messages:
* Messages come in hundreds of different tyes and may contain many different types of data.
* In addition to default message types, you can define your own custom message types.

#### Publishers
```sh
# ROS publisher definition
ros::Publisher pub1 = n.advertise<message_type>("/topic_name", queue_size);
# Publish a ROS message
pub.publish(msg);
```
The `pub1` object is a publisher object instantiated from the ros::Publisher class. This object allows you to publish messages by calling the `publish()` function.

To communicate with ROS master in C++, you need a **NodeHandle**. The node handle `n` will fully initialize the node.

The `advertise()` function is used to communicate with ROS and inform that you want to publish a message on a given topic name. The `"/topic_name"` indicates which topic the publisher will be publishing to.

The message_type is the type of message being published on "/topic_name". For example, the string message data type in ROS is `std_msgs::String`.

The `queue_size` indicates the number of messages that can be stored in a queue. A publisher can store messages in a queue until the messages can be sent. If the number of messages stored exceeds the size of the queue, the oldest messages are dropped.

##### Quiz: Publisher
Assume that a queued message is typically picked up in an average time of 1/10th of a second with a standard deviation of 1/20th of a second, and your publisher is publishing at a frequency of 10Hz. Of the options below, which would be the best setting for queue_size?
* `queue_size=2`

#### Subscribers
A subscriber enables your node to read messages from a topic, allowing useful data to be streamed to the node.
```sh
# ROS subscriber definition
ros::Subscriber sub1 = n.subscribe("/topic_name", queue_size, callback_function);
```
The `sub1` object is a subscriber object instantiated from the ros::Subscriber class. This object allows you to subscribe to messages by calling the `subscribe()` function.

To communicate with the ROS Master in C++, you need a **NodeHandle**. The node handle `n` will initialize the node.

The `"/topic_name"` indicates the topic to which the Subscriber should listen.

The `queue_size` determines the number of messages that can be stored in a queue. If the number of messages published exceeds the size of the queue, the oldest messages are dropped. As an example, if the `queue_size` is set to 100 and the number of messages stored in the queue is equal to 100, we will have to start deleting old messages to make room in the queue for new messages. This means that we are unable to process messages fast enough and we probably need to increase the `queue_size`.

The `callback_function` is the name of the function that will be run each incoming message. Each time a message arrives, it is passed as an argument to `callback_function`. Typically, this function performs a useful action with the incoming data. Note that unlike service handler functions, the `callback_function` is not required to return anything.

##### Quiz: Subscriber
Which of the following ROS nodes would likely need a Subscriber?
* A node for an autonomous vehicle that implements pedestrian detection using camera data.
* A controller node for a lunar rover which implements the actuation of the throttle and brake given target velocities as input

#### Services
For request response interaction, ROS provides what are called **services**. Like topics, services allow the passing of messages between nodes but using request and response messages. Once the requests have been handled successfully by functions or methods, the node providing the service sends a message back to the requester node.

##### ServiceServer
```sh
# ROS ServiceServer definition
ros::ServiceServer service = n.advertiseService(`service_name`, handler);

```
In ROS, the service class name `ServiceServer` comes from the file name where the service definition exists. Each service provides a definition in a `.srv` file; this is a text file that provides the proper message type for both requests and responses.

The `advertiseService()` allows you to communicate with ROS through the node handle `n` and inform ROS that you want to create a service.

The `service_name` is the name given to the service. Other nodes will use this name to specify the service to which they are sending requests.

The `handler` is the name of the function or method that handles the incoming service message. This function is called each time the service is called, and the message from the service call is passed to the `handler` function as an argument. The `handler` should return an appropriate service response message.

Services can be called directly from the command line:
```sh
rosservice call service_name "request"
```
After calling the service, you will wait for an answer.

##### ServiceClient
ROS client provides the interface for sending messages to the service:
```sh
# ROS ServiceClient definition
ros::ServiceClient client = n.serviceClient<package_name::service_file_name>("service_name");
# Request a service
client.call(srv);
```
The `client` object is instantiated from the ros::ServiceClient class. This object allows you to request services by calling the `client.call()` function.

To communicate with the ROS Master in C++, you need a **NodeHandle**. The node handle `n` will initialize the node.

The `package_name::service_file_name` indicates the name of the service file located in the `srv` directory of the package.

The `service_name` argument indicates the name of the service which is defined in the service server node.

#####

##### Quiz: Services
The following statements are true about ROS services:
* Services are similar to topics in that they faciliate the passing of messages between nodes.
* Services use a request-response message passing scheme, rather than the pub-sub method used with topics.
* A request message on a service might actually trigger a new sensor measurement, like a new camera image, with settings like exposure time specified in the message.

Which of the following ROS nodes might best be implemented using a service?
* A node for a lunar rover that shuts down a robotics arm by folding the arm and killing all related processes.
* A node that sets a given parameter on request. For example, a node in turtlesim that sets the pen color in the turtlesim window.
* A node which executes movement for a robotics arm, checking that the arm joints are within specified bounds.

#### Compute graph
The **compute graph** is useful for understanding what nodes exist and how they communicate with one another. ROS provides a tool called *RQT* graph for showing the compute graph of a system.

#### Environment
##### Quiz: Setting up the ROS environment
* `source` command: Executes the bash script within the existing environment.
* `./` command: Environment used by the executed command is destroyed wen script is done running.

### Turtlesim
#### Overview
The use of turtles in robotics goes way back to the 1940s. Wiliam Grey Walter created some of the first autonomous devices, turtle robots which he called Elmer and Elsie.

In the 1960s at MIT, Seymour Papert used turtle robots in robotics education. His robots could perform a few basic functions. They could move forward and backwards by a giving distance, rotate by a given angle and drop the the pen from their bellies allowing them to throw as they move.

##### Quiz: commands
![](images/turtlesim_quiz.png)

What commands might this program have executed to get the turtle to this point?

* Move forward 40 steps.
* Turn right 90 degrees.

#### Run
```sh
# Terminal 1: Starting the master process
roscore
# Terminal 2: Running turtlesim_node from turtlesim package
rosrun turtlesim turtlesim_node
# Terminal 3: Running turtle_teleop_key node from turtlesim package
rosrun turtlesim turtle_teleop_key
```

#### List nodes
```sh
# Termina 4: Listing all active nodes
rosnode list
```
* `/rosout`: It is a node that is automatically launched by ROS master. It's responsible for aggregating, filtering and recording log messages to a text file.
* `/teleop_turtle`: It is our keyboard teleop node. Notice that its notnamed turtle_teleop_key. There's no requirement that a node's broadcasted name is he same as the name of it's associated executable.
* `/turtlesim/`: It is the node name associated with the turtlebot_sim node.

#### List topics
```sh
# Terminal 4: Listing all topics
rostopic list
```
* `/rosout_agg`: Aggregated feed of messages published to /rosout.
* `/turtle1/cmd_vel`: Topic on which velocity commands are sent/received. Publishing a velocity message to this topic will command turtle1 to move.
* `/turtle1/color_sensor`: Each turtle in turtlesim is equipped with a color sensor, and readings from the sensor are published to this topic.
* `/turtle1/pose`: The position and orientation of turtle1 are published to this topic.

#### Get topic info
```sh
# Terminal 4: Get information about a s specifics topic
rostopic info /turtle1/cmd_vel
```
There are two nodes registered on this topic. One publisher, the `teleop_turtle` node, and one subscriber, the `turtlesim` node. Additionally, we see that the type of message used on this topic is `geometry_msgs/Twist`.

#### Message information
```sh
# Terminal 4: Show message information
rosmsg info geometry_msgs/Twist
```
`Twist` message consists nothing more than two `Vector3` messages. One for linear velocity, and another for angular velocity, with each velocity component (x,y,z) represented by a float64.

**rosed** is a simple bash command that allows you to easily view and edit files in your ROS environment.
```sh
# Terminal 4: View and edit the name of the package containing the message in question
rosed geometry_msgs Twist.msg
```

#### Echo a topic
```sh
# Terminal 4: Echo messages on a topic in real time
rostopic echo /turtle1/cmd_vel
```
If we then command the turtle to move from the `turtle_teleop_key` window, we will be able to see the output message in real-time!

### Catkin
#### Workspaces
A catkin workspace is a top-level directory where you build, install, and modify catkin packages. The workspace contains all of the packages for your project, along with several other directories for the catkin system to use when building executables and other targets from your source code.

#### Packages
ROS software is organized and distributed into packages, which are directories that might contain source code for ROS nodes, libraries, datasets, and more. Each package also contains a file with build instructions - the CMakeLists.txt file - and a package.xml file with information about the package. Packages enable ROS users to organize useful functionality in a convenient and reusable format.

#### Create a catkin workspace
```sh
# Create a top level catkin workspaces directory and a sub-directory named src
mkdir -p ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/src
# Navigate to the src directory
cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/src
# Initialize the catkin workspace (don't forget to source your environment)
souce /opt/ros/kinetic/setup.bash
catkin_init_workspace
# Return to top level directory
cd ..
# Build the workspace
catkin_make
```
You now have two new directories. The aptly named `build` directory is the build space for C++ packages and, for the most part, you will not interact with it. The `devel` directory does contain something of interest, a file named `setup.bash`. This setup.bash script must be sourced before using the catkin workspace:
```sh
source devel/setup.bash
```
You may find helpful the catkin workspace conventional directory structure as described in the ROS Enhancement Proposal (REP) 128 by clicking [here](https://www.ros.org/reps/rep-0128.html).

#### Add a package
```sh
# Add existing package
cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/src
git clone -b first_interaction https://github.com/udacity/RoboND-simple_arm/ simple_arm
cd .. && catkin_make
```

#### Roslaunch
`roslaunch` allows you to do the following:
* Launch the ROS master and multiple nodes with one simple command
* Set default parameters on the parameter server
* Automatically re-spawn processes that have died

To use `roslaunch`, you must first make sure that you have sourced the ROS environment as well as your workspace has been built and sourced:
```sh
source /opt/ros/kinetic/setup.bash
cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws
catkin_make
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch
```

##### Example: arm_mover
Make sure that the catkin_ws is compiled.
```sh
# Terminal 1: launch the robot arm
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
roslaunch simple_arm robot_spawn.launch
```
```sh
# Terminal 2: display the camera image stream
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
rqt_image_view /rgb_camera/image_raw
```
```sh
# Terminal 3: Echo /rosout topic
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
rostopic echo /rosout
```
```sh
# Terminal 4: Verify that the node and service have indeed launched
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
# /arm_mover node
rosnode list
# /arm_mover/safe_move service
rosservice list
# Mode the robot arm to move the camera and see something more than a gray image
rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"
# You will see the following message in Terminal 3:
# msg: "j2 is out of bounds, valid range (0.00,1.00), clamping to: 1.00"
# In order to increase joint 2's maximum angle:
rosparam set /arm_mover/max_joint_2_angle 1.57
# And move again
rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"
```

##### Example: look_away
Make sure that the catkin_ws is compiled.
```sh
# Terminal 1: launch the robot arm with the look_away node
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
roslaunch simple_arm robot_spawn_look_away.launch
```
```sh
# Terminal 2: display the camera image stream
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
rqt_image_view /rgb_camera/image_raw
```
```sh
# Terminal 3: Move the robot arm directly up towards the sky
source cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
rosservice call /arm_mover/safe_move "joint_1: 0
joint_2: 0"
```
Quiz:
* The arm moves to point towards the sky
* The arm points back to toward the blocks

#### Rosdep
ROS packages have two types of dependencies: build dependencies and run dependencies.

The `rosdep` tool will check for a package's missing dependencies, download them, and install them:
```sh
source ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/devel/setup.bash
rosdep check <package_name>
```

This gives you a list of the system dependencies that are missing, and tells you where to get them.

To have `rosdep` install packages, invoke the following command from the root of the catkin workspace:
dependencies, download them, and install them:
```sh
rosdep install -i <package_name>
```

#### Dive deeper into pakcages
The syntax for creating a catkin package is:
```sh
# At src directory
cd ~/udacity_robotics_sw_engineer/3-ros_essentials/catkin_ws/src
catkin_create_pkg <package_name> [dependency1 dependency2 ...]
```
The name of your package is arbitrary but you will run into trouble if you have multiple packages with the same name in your catkin workspace. Try to make it descriptive and unique without being excessively long.

Example:
```sh
catkin_create_pkg first_package
```
Navigating inside our newly created package reveals that it contains just two files: `CMakeLists.txt` and `package.xml`. This is a minimum working catkin package. It is not very interesting because it doesn't do anything, but it meets all the requirements for a catkin package. One of the main functions of these two files is to describe dependencies and how catkin should interact with them.

ROS packages have a conventional directory structure:
* scripts (python executables)
* src (C++ source files)
* msg (for custom message definitions)
* srv (for service message definitions)
* include -> headers/libraries that are needed as dependencies
* config -> configuration files
* launch -> provide a more automated way of starting nodes

Other folders may include:
* urdf (Universal Robot Description Files)
* meshes (CAD files in .dae (Collada) or .stl (STereoLithography) format)
* worlds (XML like files that are used for Gazebo simulation environments)

### URDF
**URDF** or Unified Robot Description Format uses [XML](https://www.w3schools.com/xml/xml_whatis.asp) markup language. We can use a URDF file to define a robot model, its kinodynamic properties, visual elements and even model sensors for the robot. URDF can only describe a robot with rigid links connected by joint in a chain or tree structure. It cannot describe a robot with flexible or parallel links.

Since we use URDF files to describe several robot and environmental properties, the files tend to be long and tedious. This is why we use Xacro (XML Macros) to divide our single URDF file into multiple Xacro files. While the syntax remains the same, we can now divide our robot description into smaller subsystems.

Since URDF (and Xacro) files are basically XML, they use tags to define robot geometry and properties. The most important and commonly used tags with their elements are described below:
* <robot></robot>
* <link></link>
* <joint></joint>

Example of a link:
```xml
<!-- Each rigid link in a robot must have this tag associated with it -->
<link name="link_1">
  <!-- The inertial properties of the link are described within this tag -->
  <inertial>
    <!-- This is the pose of the inertial reference frame, relative to the link reference frame -->
    <!-- The origin of the inertial reference frame needs to be at the center of gravity -->
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <!-- The mass of the link is represented by the value attribute of this element -->
    <mass value="${mass1}"/>
    <!-- The 3x3 rotational inertia matrix, represented in the inertia frame -->
    <!-- Because the rotational inertia matrix is symmetric, only 6 above-diagonal elements of this matrix are specified here, using the attributes ixx, ixy, ixz, iyy, iyz, izz -->
    <inertia ixx="30" ixy="0" ixz="0" iyy="50" iyz="0" izz="50"/>
  </inertial>
  <!-- This element specifies the appearance of the object for visualization purposes -->
  <visual>
    <!-- The reference frame of the visual element with respect to the reference frame of the link -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- The shape of the visual object -->
    <geometry>
      <mesh filename="package://kuka_arm/meshes/kr210l150/visual/link_1.dae"/>
    </geometry>
    <!-- The material of the visual element -->
    <material name="">
      <color rgba="0.75294 0.75294 0.75294 1"/>
    </material>
  </visual>
  <!-- The collision properties of a link -->
  <collision>
    <!-- The reference frame of the collision element, relative to the reference frame of the link -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- See the geometry description in the above visual element -->
    <geometry>
      <mesh filename="package://kuka_arm/meshes/kr210l150/collision/link_1.stl"/>
    </geometry>
  </collision>
</link>
```
The `<link>` tag has many more optional elements that can be used to define other properties like color, material, texture, etc.

Example of a joint:
```xml
<!-- This tag typically defines a single joint between two links in a robot -->
<joint name="joint_2" type="revolute">
  <!-- This is the transform from the parent link to the child link -->
  <!-- The joint is located at the origin of the child link -->
  <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
  <!-- Name of the Parent link for the respective joint -->
  <parent link="link_1"/>
  <!-- Name of the child link for the respective joint -->
  <child link="link_2"/>
  <!-- Defines the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints -->
  <!-- Fixed and floating joints do not use the axis field -->
  <axis xyz="0 1 0"/>
</joint>
```
Other optional elements under the `<joint>` tag. And the type of joints you can define using this tag include:
| NAME       | DESCRIPTION                                                                            |
|------------|----------------------------------------------------------------------------------------|
| Fixed      | Rigid joint with no degrees of freedom. Used to weld links together.                   |
| Revolute   | A range-limited joint that rotates about an axis.                                      |
| Prismatic  | A range-limited joint that slides along an axis.                                       |
| Continuous | Similar to Revolute joint but has no limits. It can rotate continuously about an axis. |
| Planar     | A 2D Prismatic joint that allows motion in a plane perpendicular to an axis.           |
| Floating   | A joint with 6 degrees of freedom, generally used for Quadrotors and UAVs.             |

Example of a robot:
```xml
<?xml version="1.0"?>
<!-- This is a top level tag that contains all the other tags related to a given robot -->
<robot name="two_link_robot">
  <!--Links-->
  <link name="link_1">
    <!-- ... -->
  </link>
  <link name="link_2">
    <!-- ... -->
  </link>
  <!-- ... -->
  <!--Joints-->
  <joint name="joint_1" type="continuous">
    <!-- ... -->
  </joint>
  <!-- ... -->
</robot>
```

### Sensors
#### Camera
Cameras are one of the most common sensors in Robotics. They capture information that is easily interpreted by humans at a high resolution compared to other sensors. Every image captures millions of pixels. To extract depth information from camera images, people have started using stereo cameras. These work like your eyes do and are able to estimate distances to objects.

#### Lidar
Lidar stands for Light Detection and Ranging. It uses arrays of lasers to sense "point cloud" models of the environment. By measuring thousands of millions of times per second, lidar builds an accurate model of the world. However, the resolution is not nearly as high as that of a camera.

### RViz
RViz stands for ROS Visualization. RViz is our one-stop tool to visualize all three core aspects of a robot: perception, decision-making, and actuation.

While Gazebo is a physics simulator, RViz can visualize any type of sensor data being published over a ROS topic: camera images, point clouds, ultrasonic measurements, lidar data, inertial measurements, and more. This data can be a live stream directly from the sensor or pre-recorded data stored as a **bagfile**.

You can also visualize live joint angle values from a robot and hence construct a real-time 3D representation of any robot.

```sh
# Make sure you have a roscore to run rviz
# Terminal 1:
roscore
# Terminal 2:
rosrun rviz rviz
```
RViz by default starts with two fixed property fields that cannot be removed: **Global Options** and **Global Status**. One governs simple global settings, while the other detects and displays useful status notifications.
