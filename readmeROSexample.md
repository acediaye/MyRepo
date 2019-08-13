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

## ROS C++ publisher
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

## ROS C++ subscriber
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

## ROS C++ server
add_two_ints_server.cpp
service file
```bash
int64 a
int64 b
---
int64 sum
```
```bash
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
bool add (beginner_tutorials::AddTwoInts::Request  &req,					<-when service request is received, callback function is called with request as arg
		  beginner_tutorials::AddTwoInts::Response &res){
	res.sum = req.a + req.b;												<-fill in response to req arg
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;															<-return true to indicate that it has executed properly
} 
int main(int argc, char **argv){
	ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);	<-advertise(sevice_name, callback_function)
    ROS_INFO("Ready to add two ints.");
    ros::spin(); 
    return 0;
}
```

## ROS C++ client
add_two_ints_client.cpp
```bash
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib> 
int main(int argc, char **argv){
	ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3){
		ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");	<-serviceClient<service_type>(service_name)
    beginner_tutorials::AddTwoInts srv;																<-create service request contents service.request
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv)){																			<-call service with client.call(service)
		ROS_INFO("Sum: %ld", (long int)srv.response.sum);											<-response is stored in service response
    }
    else{
		ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    } 
    return 0;
}
```

## ROS actions (actionlib)
					    (action)
[node 1/action client]----goal-->[node 2/action server]
					  --cancel-->
					  <--status--
					  <--result--
					  <-feedback-
action file
```bash
goal
---
result
---
feedback
```

* similar to service calls, but provide possibility to
  * cancel the task preemptively
  * receive feedback on the progress
* best way to implement interfaces to time extended, goal oriented behaviors
* similar in structure to services, action are defined in *.action files
* internally actions are implemented with a set of topics

## ROS chart
|        | Parameters | Dynamic Reconfigure | Topics | Services | Actions |
|        | --- | --- | --- | --- | --- |
|description| global constant parameters | local, changeable parameters | coninuous data streams | blocking call for processing a request | non blocking preemptable goal oriented tasks
|application| constant settings | tuning parameters | one way continuous data flow | short triggers or calculations | task executions and robot actions |
|examples| topic names, camera setting, calibration data, robot setup | controller parameters | sensor data, robot state | trigger change, request state, compute quantity | navigation, grasping, motion execution |
