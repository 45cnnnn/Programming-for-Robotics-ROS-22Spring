# Programming-for-Robotics-ROS-22Spring

+ Lecturers: Edo Jelavić, Mayank Mittal, Prof. Marco Hutter

+  [**[Webpage](https://rsl.ethz.ch/education-students/lectures/ros.html)**]

### Abstract

This course gives an introduction to the Robot Operating System (ROS) including many of the available tools that are commonly used in robotics. With the help of different examples, the course should provide a good starting point for students to work with robots. They learn how to create software including simulation, to interface sensors and actuators, and to integrate control algorithms.

### Objective:

+ ROS architecture: Master, nodes, topics, messages, services, parameters and actions
+ Console commands: Navigating and analyzing the ROS system and the catkin workspace
+ Creating ROS packages: Structure, launch-​files, and best practices
+ ROS C++ client library (roscpp): Creating your own ROS C++ programs
+ Simulating with ROS: Gazebo simulator, robot models (URDF) and simulation environments (SDF)
+ Working with visualizations (RViz) and user interface tools (rqt)
+ Inside ROS: TF transformation system, time, bags
+ Introduction to ROS2

## Lecture 1

+ Topic
	ROS architecture & philosophy; ROS master, nodes, and topics; Console commands; Catkin workspace and build system; Launch-files; Gazebo simulator; Programming Tools

#### ROS Nodes

```shell
rosrun package_name node_name
rosnode list
rosnode info node_name
```
#### ROS Topics

```shell
rostopic list
rostopic echo /topic
rostopic info /topic
```

#### ROS Messages

+ Data structure defining the type of a topic
+ Defined in **.msg*  files
```
rostopic type /topic
rostopic pub /topic type data
rostopic pub /chatter std_msgs/String "data: 'ETH Zurich ROS Course'"
```
[example: PoseStamped.msg]
> **[ROS Wiki Message files](http://wiki.ros.org/Messages)**

#### ROS Nodelets

> **[ROS Wiki Nodelet](http://wiki.ros.org/nodelet) **

#### ROS Workspace Environment

```
source /opt/ros/noetic/setup.bash
```

##### catkin build System

```
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
Clean the entire build and devel space with
```
catkin clean
```
The catkin workspace setup can be checked with
```
catkin config
```
For example, to set the CMake build type to Release (or Debug etc.), use:
```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Symlink a new package to your catkin workspace
```
ln -s ~/git/ros_best_practices/ ~/catkin_ws/src/
```

#### ROS Launch

+ lauch is a tool for launching multiple nodes (as well as setting parameters)
+ Are written in XML as **.launch* files
+ If not yet running, launch automatically starts a roscore
```
roslaunch file_name.launch
roslaunch package_name file_name.launch
```
> **[ROS Wiki Launch files](http://wiki.ros.org/roslaunch/XML) **
> **[Roslaunch tips for large projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) **

```XML
<?xml version="1.0"?>
<launch>

  <!-- Launch ROS Package Template Node -->
	<node pkg="ros_package_template" type="ros_package_template" name="ros_package_template" output="screen">
		<rosparam command="load" file="$(dirname)/../config/default.yaml" />
	</node>

</launch>
```

```XML
<!-- Create a re-usable launch files with <arg> tag, which works like a parameter -->
<arg name="use_sim_time" default="true"/>

<!-- Use arguments in launch file with $(arg use_sim_time)-->
<group if="$(arg use_sim_time)">
	<param name="/use_sim_time" value="true" />
</group>

<!-- Include other launch files -->
<!-- Find the system path to other packages with $(find package_name)-->
<include file="$(find gazebo_ros)/launch/empty_world.launch">
	<arg name="world_name" value="$(find gazebo_plugins)/test/test_worlds/$(arg world).world"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="physics" value="$(arg physics)"/>
</include>
```

#### Gazebo Simulator

> > **[Gazebo tutorials](http://gazebosim.org/tutorials)**



## Lecture 2

+ Topic
	ROS package structure; ROS C++ client library (roscpp); ROS subscribers and publishers; ROS parameter server; RViz visualization

#### ROS Packages

```
.
├── CMakeLists.txt
├── config
│   └── default.yaml                                   //(paramter files)
├── include
│   └── ros_package_template                           //(package name)
│       ├── Algorithm.hpp
│       └── RosPackageTemplate.hpp
├── launch
│   ├── ros_package_template.launch
│   └── ros_package_template_overlying_params.launch
├── LICENSE
├── package.xml
├── README.md
├── src
│   ├── Algorithm.cpp
│   ├── RosPackageTemplate.cpp
│   └── ros_package_template_node.cpp
└── test
    ├── AlgorithmTest.cpp
    └── test_ros_package_template.cpp
```
+ package.xml
	+ Package name
	+ Version Number
	+ Authors
	+ **Dependencies on other packages**
	+ ...
+ CmakeLists.txt
	+ see example 
+ pacakeg_name_msgs folder


#### ROS C++ Client Library (roscpp)

+ Initialization and spinning
	+ [ros::spin() and ros::spinOnce()](:https://answers.ros.org/question/11887/significance-of-rosspinonce/)
	
+ Node Handle
	+ Default(public) node handle
	```
	nh_ = ros::NodeHandle();                        /namespace/topic
	```
	+ Private node handle
	```
	nh_private_ = ros::NodeHandle("~");             /namespace/node/topic
	```
	+ Namespace node handle
	```
	nh_eth_ = ros::NodeHandle("eth");               /namespace/eth/topic
	```
	+ Global node handle (not recommanded)
	
+ Logging
	+ [rosconsle](https://wiki.ros.org/rosconsole)
	+ [Logging](https://wiki.ros.org/roscpp/Overview/Logging) 
	
+ [Subscriber and Publisher](https://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)

+ Object Oriented Programming

+ Paramters
    ```c++
    ros::NodeHandle nodeHandle("~");
    std::string topic;
    if (!nodeHandle.getParam("topic", topic)) {
        ROS_ERROR("Could not find topic parameter!");
    }
    ROS_INFO_STREAM("Read topic: " 
    ```
#### ROS Paramter Server 
+ [Parameters](https://wiki.ros.org/rosparam)
+ YAML files
+ launch API
+ Command line tools
	```
	rosparam list
	rosparam get parameter_nam
	rosparam set parameter_name value
	```

#### [Rviz](https://wiki.ros.org/rviz)

#### Exercise 2

+ [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan)
+ [Introduction to Working With Laser Scanner Data](http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData)
+ [LaserScan Msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)



## Lecture 3

+ Topic
	TF Transformation System; rqt User Interface; Robot models (URDF); Simulation descriptions (SDF)

#### [TF Transformation System](http://wiki.ros.org/tf2)
+ View Frames
+ C++ API
	+ [Writing a tf2 listener](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20(C++))
	+ [Using stamped datatypes with tf2::MessageFilter](http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2%3A%3AMessageFilter)
	
#### rqt

+ User interface
+ rqt_image_view
+ rqt_multiplot
+ rqt_graph
+ rqt_console
+ rqt_logger_level

#### Robots Models

+ Unified Robot Description Format (URDF)
+ Simulation Decription Format (SDF)



## Lecture 4

+ Topic
	ROS services; ROS actions (actionlib); ROS time; ROS bags; Debugging strategies; Introduction to ROS2

#### [ROS services](https://wiki.ros.org/Services)

```
rosservice list
rosservice type /service_name
rosservice call /service_name args

rossrv show ros_tutorials/TwoInts
```
+ .srv files
+ [C++ API](https://wiki.ros.org/roscpp/Overview/Services)

#### ROS actions (actionlib)

#### ROS time

#### ROS bags

#### Debug Strategies

#### Intro to ROS 2

#### Exercise 4

+ [ekf_localization_node](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html)


## Lecture 5

+ Topic
	Case study: Using ROS in complex real world applications

