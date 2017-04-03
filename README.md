# beginner_tutorials
UMD ENPM808X ROS (Robot Operating System) Beginner Tutorials

## Overview
This project contains a ROS package called "beginner_tutorials."  The package
and code for the programs in the package is based on the Core ROS Tutorials found at http://wiki.ros.org/ROS/Tutorials.

## Pre-requisites
This ROS package has been built and tested for the Indigo-Igloo release of ROS.
In order to build and use it, you will need to have ROS Indigo-Igloo installed on your system, along with the ROS dependencies identified in the package.xml manifest file (roscpp, rospy, and std_msgs).  The instructions in this README.md file assume that you are familiar with ROS and the ROS catkin build system.

## Import tutorials into your ROS catkin workspace
To import the project into your catkin workspace, clone or download it into the src subdirectory of your catkin workspace directory.  Once this package is
part of your catkin workspace, it will build along with any other packages you
have in that workspace using the "catkin_make" command executed at the top-level
directory of your catkin workspace.

## Running the tutorial programs
This ROS package beginner_tutorials contains two programs, a ROS publisher called "listener" and a ROS subscriber called "publisher."  To run them, first make sure that the catkin workspace has been added to your local ROS configuration by sourcing the build/setup.bash script in each terminal window from which you will invoke one of the programs.  Then, use the command
"rosrun beginner_tutorials listener" to run the subscriber program, and
"rosrun beginner_tutorials talker" to run the publisher program.

Be sure that you have a roscore already executing when you invoke the listener
or talker programs, the same as you would for running any other ROS nodes.
