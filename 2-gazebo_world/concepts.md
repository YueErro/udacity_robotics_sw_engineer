# Gazebo world concepts

## Table of contents
* [What is Gazebo?](#what-is-gazebo)
* [Gazebo features](#gazebo-features)
* [Gazebo components](#gazebo-components)
  * [Gazebo server](#gazebo-server)
  * [Gazebo client](#gazebo-client)
  * [World files](#world-files)
  * [Models files](#models-files)
  * [Environment Variables](#environment-variables)
  * [Plugins](#plugins)

### What is Gazebo?
Gazebo is a physics-based, high fidelity 3D simulator for robotics. Gazebo provides the ability to accurately simulate one or more robots in complex indoor and outdoor environments filled with static and dynamic objects, realistic lighting, and programmable interactions.

Gazebo facilitates robotic design, rapid prototyping, testing, and simulation of real-life scenarios.

```sh
gazebo
```

### Gazebo features

1. Dynamics Simulation: Model a robot's dynamics with a high-performance physics engine.
2. Advanced 3D Graphics: Render your environment with high-fidelity graphics, including lighting, shadows, and textures.
3. Sensors: Add sensors to your robot, generate data, and simulate noise.
4. Plugins: Write a plugin to interact with your world, robot, or sensor.
4. Model Database: Download a robot or environment from Gazebo library or build your own through their engine.
5. Socket-Based Communication: Interact with Gazebo running on a remote server through socket-based communication.
6. Cloud Simulation: Run Gazebo on a server and interact with it through a browser.
7. Command Line Tools: Control your simulated environment through the command line tools.

### Gazebo components

#### Gazebo server
`gzserver` performs most of the heavy-lifting for Gazebo. It is responsible for parsing the description files related to the scene we are trying to simulate, as well as the objects within. It then simulates the complete scene using a physics and sensor engine.

#### Gazebo client
`gzclient` on the other hand provides the very essential Graphical Client that connects to the gzserver and renders the simulation scene along with useful interactive tools.

#### World files
A **world** file in Gazebo contains all the elements in the simulated environment. These elements are your robot model, its environment, lighting, sensors, and other objects. You have the ability to save your simulation to a world file that usually has a `.world` extension.

```sh
gazebo <sdf_file>.world
```

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <physics type="ode">
      ...
    </physics>
    <scene>
      ...
    </scene>
    <model name="box">
      ...
    </model>
    <model name="sphere">
      ...
    </model>
    <light name="spotlight">
      ...
    </light>
  </world>
</sdf>
```

#### Model files
To include a **model** file of a robot or any other model inside your world file, you can add the following code to the **world’s SDF** file:

```xml
<include>
  <uri>model://model_file_name</uri>
</include>
```

A **visual** aspect of an object is it's graphical representation and does not affect the physics simulation.

A **collision** aspect of an object is used by the physics engine for collision checking.

#### Environment variables
There are many environment variables that Gazebo uses, primarily to locate files (world, model, …) and set up communications between **gzserver** and **gzclient**.

`GAZEBO_MODEL_PATH`: List of directories where Gazebo looks to populate a model file.

#### Plugins
To interact with a world, model, or sensor in Gazebo, you can write plugins.
