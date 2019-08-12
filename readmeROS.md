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

