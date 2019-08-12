# Learn how to ROS

https://www.youtube.com/watch?v=0BxVPCInS3M
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

## ROS create package with name and cpp flag
```bash
$ catkin_create_pkg pkg_name roscpp
```

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
GUI shows all nodes and topics and how they are connected 
```bash 
$ rqt_graph
```