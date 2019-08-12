# Learn how to ROS CMAKE

## Simple ROS cmake
```bash
cmake_minimum_required(VERSION 2.8.3)
project(husky_highlevel_controller)			<-use same name as in package.xml
add_definitions(--std=c++11)				<-use c++11 compiler

find_package(catkin REQUIRED COMPONENTS		<-list packages that your package requires to build, have to be listed in package.xml 
	roscpp
	sensor_msgs
}

catkin_package(								<-specify build export info
	INCLUDE_DIRS include
	# LIBRARIES
	CATKIN_DEPENDS roscpp sensor_msgs
	# DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})			<-specify locations of header files

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)		<-declare a c++ executable

target_link_liraries(${PROJECT_NAME} ${catkin_LIBRARIES})	<-specify libraries to link the executable against
```

## ROS C++ server
talker.cpp
```bash
#include "ros/ros.h"									<-ROS main header file
#include "std_msgs/String.h"
int main(int argc, char** argv){
	ros::init(argc, argv, "talker");					<-ros initialize
	ros::NodeHandle n;									<-access point for communications with ROS system
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);		<-topic chatter, need matching name, queue of 100
	ros::Rate loop_rate(10);							<-run loops at desired frequency in Hz
	int count =0;
	while(ros::ok()){									<-returns false if SIGINT is received (ctrl c) or ros::shutdown()
		std_msgs::String msg;
		msg.data = "hello world " + std::to_string(count);
		ROS_INFO_STREAM(msg.data);						<-logs messages to filesystem
		chatter_pub.publish(msg);
		ros::spinOnce();								<-process incoming messages via callbacks
		loop_rate.sleep();								<-sleep for the remaining time to get desired Hz
		count++;
	}
	return 0;
}
```

## ROS C++ client
lisener.cpp
```bash
#include "ros/ros.h"
#include "std_msgs/String.h"
void chatterCallback(const std_msgs::String::ConstPtr& msg){				<-when message is received, callback function is called
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char** argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 100, chatterCallback);		<-topic, queue_size, callback_function
	ros::spin();															<-processes callback until node shutdown
	return 0;
}
```

## logging
supports both printf and stream style formatting
```c++
ROS_INFO("Result: %d", result);
ROS_INFO_STREAM("Result: " << result);
```
5 loging levels

* debug
* info
* warn
* error
* fatal

