# beginner_tutorials2
UMD ENPM808X ROS (Robot Operating System) Beginner Tutorials New! Improved!

## Overview
This project contains a ROS package called "beginner_tutorials."  The package
and code for the programs in the package is based on the Core ROS Tutorials found at http://wiki.ros.org/ROS/Tutorials.  Additions to the "talker" (example Publisher) and "listener" (example Subscriber)
programs demonstrate the use of services, message logging, and command line arguments.  A roslaunch
XML file demonstrates the use of this capability.

## Pre-requisites
This ROS package has been built and tested for the Indigo-Igloo release of ROS.
In order to build and use it, you will need to have ROS Indigo-Igloo installed on your system, along with the ROS dependencies identified in the package.xml manifest file (roscpp, rospy, and std_msgs).  The instructions in this README.md file assume that you are familiar with ROS and the ROS catkin build system.

## Import tutorials into your ROS catkin workspace
To import the project into your catkin workspace, clone or download it into the src subdirectory of your catkin workspace directory.  Once this package is part of your catkin workspace, it will build along with any other packages you have in that workspace using the "catkin_make" command executed at the top-level directory of your catkin workspace.

## Running the tutorial programs
This ROS package beginner_tutorials2 contains two programs, a ROS publisher called "listener" and a ROS subscriber called "talker."  To run them, first make sure that the catkin workspace where you have placed the package has been added to your local ROS configuration by sourcing the build/setup.bash script in each terminal window from which you will invoke one of the programs.  Then, use the command
"rosrun beginner_tutorials2 listener" to run the subscriber program, and
"rosrun beginner_tutorials2 talker" to run the publisher program.

Be sure that you have a roscore already executing when you invoke the listener
or talker programs, the same as you would for running any other ROS nodes.

## Adjusting the tutorial programs using service requests
Both the talker and the listener nodes can be adjusted while they are running using service calls.

- listener: provides the set_listener_logging_level service to adjust the level at which the listener node notifies the logging service of the chatter messages it has received.  The level can be set to DEBUG, INFO, WARN, ERROR, or FATAL.  Example:

    - rosservice call /set_listener_log_level WARN

- talker: provides the set_talker_frequency service to adjust the level at which the talker node publishes messages to the chatter topic.  The frequency, specified as a messages per second rate, can be set to any non-zero, positive value.  Example:

    - rosservice call /set_talker_frequency 0.5
    
## Running the talker with a specified message rate
The talker can be run with an initial message rate using the command line parameter "-r <rate>" where the rate is a real number specifying the number of messages per second.  Example:

    - rosrun beginner_tutorials2 talker -r 1.5
    
## Launching the talker and listener at the same time
Since the talker and listener are typically run together, a launch file conveniently starts them both at the same time.  The launch file permits an initial message publishing rate for the talker node to be specified; if the rate argument is not specified, the launcher uses a default value.  Example:

    - roslaunch beginner_tutorials2 talklisten rate:=0.3
    
## Use of logging levels
The following messages may be logged by talker and listener:

- listener: DEBUG: "Listener log level " << level << " requested"
- listener: WARN: "Invalid listener logging level " << level << " requested; level remains " << listenerLoggingLevel
- listener: INFO: "Listener logging level set to " << listenerLoggingLevel
- listener: ERROR: "Invalid listener logging level " << listenerLoggingLevel
- listener: Any level selected: "I heard: " << msg->data.c_str()
- talker: INFO: "Changing talk frequency to " << req.msgsPerSecond << " messages per second"
- talker: ERROR: "Invalid talker frequency of " << req.msgsPerSecond << " requested"
- talker: FATAL: "Talker can't start due to invalid frequency"
- talker: FATAL: "Talker can't continue due to invalid frequency"
- talker: INFO: msg.data.c_str()
