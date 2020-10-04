# ROS

## Table of contents
* [What is ROS?](#what-is-ROS)
* [ROS components and features](#ros-components-and-features)
* [ROS nodes and topics](#ros-nodes-and-topics)
  * [Quiz: Nodes and Topics](#quiz-nodes-and-topics)
* [ROS message](#ros-message)
  * [Quiz: Messages](#quiz-messages)
* [ROS services](#ros-services)
  * [Quiz: Services](#quiz-services)
* [Compute graph](#compute-graph)
* [ROS environment](#ros-environment)
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

### What is ROS
ROS or Robot Operating System is a open-source software framework for robotics development.

### ROS components and features
ROS porvides a means of communicating with hardware and a way for different processes to communicate with one another via message passing.

ROS features a slick build and package managemnt system called **catkin**, allowing you to develop and deploy software with ease.

To sum up ROS components and features:
* Open-source
* Hardware abstraction of device drivers
* Communication via message passing
* Slick build and package management
* Tools for visualization, simulation and analysis
* Powerful software libraries

### ROS nodes and topics
**ROS master** maintains the registry of all the active nodes on a system. Each node can use this registry to discover other nodes and establish lines of communication. In addition, it also host what's called the parameter server.

The **parameter server** is typically used to store parameters and configuration values, that are share among the the running nodes.Nodes can also share data with one another by passing messages over what are called topics.

**Topic** is a pipe between nodes, trough which messages flow. In order to send a message on a topic, we say that the node must publish (**publisher**) to that topic. Likewise, to receive a message on a topic, a node must subscribe (**subscriber**) to that topic.

A network of nodes connected by topics is called a publish subscribe or pops up  architecture.

#### Quiz: Nodes and Topics
The following statements are true about ROS nodes and topics:
* Robots may be very different in form and function, but they all perform the same high-level tasks of perception, decision making, and actuation.
* The parameter server acts as a central repository where nodes on a system can look up parameter values.
* Nodes pass messages to one another via topics, which you can think of as a pipe connecting two nodes.
* A single node may simultaneously publish and subscribe to many topics.

### ROS message
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

#### Quiz: Messages
The following statements are true about ROS messages:
* Messages come in hundreds of different tyes and may contain many different types of data.
* In addition to default message types, you can define your own custom message types.

### ROS services
For request response interaction, ROS provides what are called **services**. Like topics, services allow the passing of messages between nodes but using request and response messages.

#### Quiz: Services
The following statements are true about ROS services:
* Services are similar to topics in that they faciliate the passing of messages between nodes.
* Services use a request-response message passing scheme, rather than the pub-sub method used with topics.
* A request message on a service might actually trigger a new sensor measurement, like a new camera image, with settings like exposure time specified in the message.

### Compute graph
The **compute graph** is useful for understanding what nodes exist and how they communicate with one another. ROS provides a tool called *RQT* graph for showing the compute graph of a system.

### ROS environment
#### Quiz: Setting up the ROS environment
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
