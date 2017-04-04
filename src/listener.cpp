/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file listener.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Mar 27, 2017 - Creation
 * @date Apr 4, 2017 - Updated
 *
 * @brief ROS Tutorials - Publisher/Subscriber - listener node
 *
 * This source file creates a subscriber node within the ROS environment as part
 * of the ROS beginner tutorials.  The base code is based on the source code provided
 * in the tutorials, and is not an original creation of Mark R. Jenkins.  The /set_listener_log_level
 * service, with the ability to change the logging level of the logging messages sent when
 * messages are received from the "chatter" service, is an original creation of Mark Jenkins.
 *
 * *
 * * BSD 3-Clause License
 *
 * Copyright (c) 2017, Mark Jenkins
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "beginner_tutorials2/ListenerLogLevel.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * It has been modified to provide a service that can change the logging level
 * of the messages received through the chatter topic
 */

// Establish startup default logging level
std::string listenerLoggingLevel = "INFO";

/**
 * @brief Service callback for setting listener logging level
 * @param req An object with a "level" attribute string that should correspond to one of the logging levels
 * @param resp An object with a "level" attribute string indicating the current logging level
 */
bool setListenerLoggingLevel(
    beginner_tutorials2::ListenerLogLevel::Request &req,
    beginner_tutorials2::ListenerLogLevel::Response &resp) {

  // Set the listener logging level to the requested level if the level is a valid level
  std::string level = req.level;
  ROS_DEBUG_STREAM("Listener log level " << level << " requested");
  if (level == "DEBUG")
    listenerLoggingLevel = "DEBUG";
  else if (level == "INFO")
    listenerLoggingLevel = "INFO";
  else if (level == "WARN")
    listenerLoggingLevel = "WARN";
  else if (level == "ERROR")
    listenerLoggingLevel = "ERROR";
  else if (level == "FATAL")
    listenerLoggingLevel = "FATAL";
  else {
    // An invalid listener logging level was requested; issue a warning and do not change level
    ROS_WARN_STREAM(
        "Invalid listener logging level " << level << " requested; level remains " << listenerLoggingLevel);
    resp.level = listenerLoggingLevel;
    return false;
  };
  // Signal that the listener logging level was changed successfully
  ROS_INFO_STREAM("Listener logging level set to " << listenerLoggingLevel);
  resp.level = listenerLoggingLevel;
  return true;
}

/**
 * @brief Subscription callback for the chatter topic; logs the chatter received at the current listener logging level
 * @param msg A message string received on the chatter topic
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  // If the current listener logging level is valid, log the chatter that was received at that level
  if (listenerLoggingLevel == "DEBUG")
    ROS_DEBUG_STREAM("I heard: " << msg-> data.c_str());
  else if (listenerLoggingLevel == "INFO")
    ROS_INFO_STREAM("I heard: " << msg->data.c_str());
  else if (listenerLoggingLevel == "WARN")
    ROS_WARN_STREAM("I heard: " << msg->data.c_str());
  else if (listenerLoggingLevel == "ERROR")
    ROS_ERROR_STREAM("I heard: " << msg->data.c_str());
  else if (listenerLoggingLevel == "FATAL")
    ROS_FATAL_STREAM("I heard: " << msg->data.c_str());
  else
    // The current listener logging level is invalid; log an error
    ROS_ERROR_STREAM("Invalid listener logging level " << listenerLoggingLevel);
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Register the chatterLoggingLevel service with the master
  ros::ServiceServer server = n.advertiseService("set_listener_log_level",
                                                 &setListenerLoggingLevel);

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
