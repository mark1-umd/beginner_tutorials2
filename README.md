# beginner_tutorials2
UMD ENPM808X ROS (Robot Operating System) Beginner Tutorials New! Improved!

## Overview
This project contains a ROS package called "beginner_tutorials."  The package
and code for the programs in the package is based on the Core ROS Tutorials found at http://wiki.ros.org/ROS/Tutorials.  Additions to the "talker" (example Publisher) and "listener" (example Subscriber)
programs demonstrate the use of the following additional ROS capabilities:

    - services (in both the talker and listener nodes)
    - message logging (in both the talker and listener nodes)
    - command line arguments (talker node only)
    - launch files (launches a talker/listener combination with an optional talker rate)
    - automatic topic recording (built into the launch file demonstration)
    - rostest/gtest (tests are executed through the catkin build system or rostest command)

## Pre-requisites
This ROS package has been built and tested for the Indigo-Igloo release of ROS.
In order to build and use it, you will need to have ROS Indigo-Igloo installed on your system, along with the ROS dependencies identified in the package.xml manifest file (roscpp, rospy, and std_msgs).  The instructions in this README.md file assume that you are familiar with ROS and the ROS catkin build system.

## Import tutorials into your ROS catkin workspace
To import the project into your catkin workspace, clone or download it into the src subdirectory of your catkin workspace directory.  Once this package is part of your catkin workspace, it will build along with any other packages you have in that workspace using the "catkin_make" command executed at the top-level directory of your catkin workspace.

## Running the tutorial programs from the command line
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
    
## Starting the talker and listener at the same time with a launch file
Since the talker and listener are typically run together, a launch file conveniently starts them both at the same time.  The launch file permits an initial message publishing rate for the talker node to be specified; if the rate argument is not specified, the launcher uses a default value.  Example invocation:

    - roslaunch beginner_tutorials2 talklisten.launch rate:=0.3

## Recording topic data with a launch file
The launch file can optionally record all published topics to a bag file; the default is to not record.  Example invocation:

    - roslaunch beginner_tutorials2 talklisten.launch record:=true

If a recording is made, the bag file will be stored in the directory ~/.ros, unless ROS_HOME is defined, in which case it will be stored there.  The bag filename is prefixed with "talklisten" and includes the date and time of the recording.

The resulting bag file can be inspected through the rosbag info command:

    - rosbag info <bagfile>
    
## Demonstrating topic data playback
Bag files can be played back, with the topic messages stored in the bag being published to the topics originally recorded in place of the original topic message publishers.  The beginner_tutorials2 listener can be used to demonstrate the playback of the chatter messages that are stored in the "talklisten" bag file recorded the talklisten.launch file.  In order to see this demonstration, the listener must be run without the talker prior to playing back the bag file contents.  The following set of commands will demonstrate this:

    - roscore [in terminal window 1]
    - rosrun beginner_tutorials2/listener [in terminal window 2]
    - rosbag play <bagfile> [in terminal window 3]
    
When the rosbag play command plays back the contents of a talklisten bag file (created using the talklisten.launch file with "record:=true" set, the listener node running in terminal window 2 will send notifications to the terminal of each "chatter" topic message it receives from the bag file being played back.  Note: Be sure to establish the local "beginner_tutorials2" package in your ROS environment for each terminal window by sourcing the <catkin workspace>/devel/setup.bash script of the catkin workspace into which you have placed the beginner_tutorials2 package.
    
## Viewing tf transform broadcasts
The talker node is programmed to broadcast coordinate frame transformations (at the same rate as it publishes the chatter messages).  The frame transformations that talker broadcasts are between a parent frame called "/world" and a child frame called "/talk".  While the talker node is running, there are several ways to observe these transforms.  First, start the talker node either from the command line or using the talklisten.launch file (don't forget to start roscore if  you use the command line, and don't forget to source your workspace's setup.bash file so ROS can find the beginner_tutorials2 package).  Next, use one or more of the following commands to observe the transforms and/or coordinate frame relationships:

    - rosrun tf tf_echo /world /talk [shows transforms on screen]
    - rosrun tf tf-echo /talk /world [shows the inverse transforms on the screen]
    
For a dynamically updatable visual depiction of the coordinate frame relationships, use:

    - rosrun rqt_tf_tree rqt_tf_tree
    
For a static visual depiction of the coordinate frame relationships, use:

    - rosrun tf view_frames
    
tf viewframes captures the current transform tree in a PDF file called "frames.pdf" in the current working directory.  Under Ubuntu Linux, the resulting file can be viewed using "evince frames.pdf"; of course, any other PDF viewer can be used as well.
    
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
