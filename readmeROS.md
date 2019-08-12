# Learn how to ROS

https://www.youtube.com/watch?v=0BxVPCInS3M \
catkin is the ROS build system to generate executables, libraries, and interfaces

## ROS setup workspace
```bash
$ source /opt/ros/melodic/setup.bash
```
create and build catkin_WS
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
everytime opening a new terminal, need to source
```bash
$ source devel/setup.bash
```
can check path
```bash
$ echo $ROS_PACKAGE_PATH
/home/user/catkin_ws/src:/opt/ros/kinetic/share
```

## ROS create package with name and dependencies flag
```bash
$ catkin_create_pkg package_name roscpp
```

* package_name
  * config (parameter files ie yaml)
  * include (c++ include headers)
  * launch (launch files)
  * src (source files)
  * test (ROS test)
  * CMakeLists.txt (cmake build file)
  * package.xml (package info)

## ROS navigating filesystem
rospack find package_name: returns path to package\
roscd package_name: change directory to package path\
rosls: lists file in package\
rosmsg: provides info on ROS message definition\
rossrv: provides info on ROS service definition

## ROS run
roscore: start ROS master

rosrun beginner_tutorial talker: run node talker from begineer_tutorial package\
rosrun beginner_tutorial listener: run node listener from beginner_tutorial package

## ROS node
rosnode list: shows all active nodes\
rosnode info talker: shows info of talker ie pub, sub, services, connections

## ROS topic
rostopic list: shows all active topics\
rostopic info chatter: shows info of chatter ie type, pub, sub\
rostopic type chatter: shows datum of topic\
rostopic echo chatter: prints what chatter topic sees\
rostopic hz chatter: shows frequency of topic chatter ie min, max, std dev\

rostopic pub chatter std_msgs/String "data: 'manually write msg on topic'"

## ROS build
catkin_make: compile and build all packages\
catkin_make package_name: for singlular package\
catkin_make clean: clean entire build and devel space\
rm -r build devel install: to delete all dependencies

## ROS folders:
src: source space where you clone, create, edit source code for the packages \
build: build space where cmake is invoked to build the packages in the source space, cache info\
devel: development space where build targets are placed(before being installed)

## ROS etc
catkin config: catkin workspace setup can be checked with

## ROS launch
roslaunch package_name file_name.launch: auto starts roscore, run multiple nodes and parameters
```bash
<launch>
	<node name="listener" pkg="beginner_tutorials" type="talker" output="screen"/>
	<node name="talker" pkg="beginner_tutorials" type="listener" output="screen"/>
</launch>
```

* launch: root element of launch file
* node: each (node) tag spcifies a node to be launched
* name: name of the node (free to choose)
* pkg: package containing the node
* type: type of the node, there must be a corresponding executable with the same name
* output: specifies where to output log messages (screen: console, log: logfile)

parameters
```bash
<arg name="arg_name" default="default_value"/>
```

## Gazebo simulator
Simulate 3d rigid body dynamics, for robots and environments, extensible with plugins
```bash
$ rosrun gazebo_ros gazebo_ros
```

## RQT
rosrun rqt_gui rqt_gui: custom interface can be setup\
rosrun rqt_image_view rqt_image_view: raw image from camera\
rosrun rqt_multiplot rqt_multiplot: visualizing numeric vlues in 2d plots\
rosrun rqt_graph rqt_graph: visualizing the ROS nodes and topic connection graph\
rosrun rqt_console rqt_console: display and filter ROS messages\
rosrun rqt_logger_level rqt_logger_level: configure logger level of ROS nodes

## RVIZ
3d visualization tool for ROS, subscribe to topics and visualizes the message contents, plugin\
add topic, fixed frame, choose topic for the display, change display options
```bash
$ rosrun rviz rviz
```

## ROS parameter
parameters can be defined in launch files or yaml files\
rosparam list: list all parameters\
rosparam get parameter_name\
rosparam set parameter_name value

## TF transformation system

* tool for keeping track of coordinate frames over time
* maintains relationship between coordinate frames in a tree structure
* lets user transform points, vectors, etc between coordinate frames at desired time
* implemented as publisher/subscriber model on topics /tf, /tf_static

tf2_msgs/TFMessage.msg\
rosrun tf tf_monitor: print transform tree\
rosrun tf tf_echo source_frame target_frame: print info about transform between 2 frames\
rosrun tf new_frames: visual graph of transform tree

## Robot Models, Unified Robot Description Format (URDF)
defines an XML format for representing a robot model\
URDF generation can be scripted with XACRO\
description consists of link elemets connected by joints

* kinematic and dynamic description
* visual representation
* collision model

## Simulation description format (SDF)
defines an XML format to describe\
SDF standard format for gazebo\
gazebo converts URDF to SDF automatically

* environments (lighting, gravity, etc)
* objects (static, dynamic)
* sensors
* robots

## ROS service
request/response communication between nodes is done with services\
rosservice list: shows all services\
rosservice type service_name: shows type of service\
rosservice call service_name args: call a service with request content\
```bash
request
---
response
```
