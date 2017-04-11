/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file talker.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Mar 27, 2017 - Creation
 * @date Apr 4, 2017 - Modified to add service to adjust the message publishing frequency
 * @date Apr 7, 2017 - Modified to broadcast a tf frame called /talk with parent /world
 *
 * @brief ROS Tutorials - Publisher/Subscriber - talker node (with added adjustable frequency)
 *
 * This source file creates a publisher node within the ROS environment as part
 * of the ROS beginner tutorials.  The bulk of the code is based on the source code provided
 * in the tutorials, and is not an original creation of Mark R. Jenkins.  The service to adjust
 * the frequency of the messages was added by Mark Jenkins, as was the tf frame broadcast.
 *
 * An enumerated message is published on the "chatter" topic at a specified frequency.
 * A service called "set_talker_frequency" accepts a "messages per second" argument and
 * adjusts the message publishing frequency accordingly.
 * A command line option "-r <value" accepts a "messages per second" value and sets
 * the initial message publishing frequency accordingly.
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

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials2/TalkerFrequency.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.  It has been
 * modified to provide an adjustable frequency at which the messages are sent.
 */

// Establish the initial message publishing frequency
double talkerFrequency = 1.0;

/**
 * @brief Service callback for setting talker frequency
 * @param req An object with a "msgsPerSecond" attribute for the frequency to send chatter messages
 * @param resp An object with a "msgsPerSecond" attribute indicating the current frequency at which chatter message are being sent
 */
bool setTalkerFrequency(
    beginner_tutorials2::TalkerFrequency::Request &req,
    beginner_tutorials2::TalkerFrequency::Response &resp) {
  // If the requested rate is valid, change the message publishing frequency
  if (req.msgsPerSecond > 0) {
    ROS_INFO_STREAM(
        "Changing talk frequency to " << req.msgsPerSecond << " messages per second");
    talkerFrequency = req.msgsPerSecond;
    resp.msgsPerSecond = talkerFrequency;
    return true;
  } else {
    // An invalid rate was specified; log an error
    ROS_ERROR_STREAM(
        "Invalid talker frequency of " << req.msgsPerSecond << " requested");
    talkerFrequency = 0.0;
    resp.msgsPerSecond = talkerFrequency;
    return false;
  }
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
  ros::init(argc, argv, "talker");

  // Set our publishing frequency, either as the default value specified for the
  // talkerFrequency, or as a rate passed as a command line argument (-r <rate>)

  // Check to see if there is a -r (rate) command line option, and set the rate
  // accordingly (ros::init should have stripped any ROS arguments, and the 0th
  // argument should be the path of the program (which we don't care about)
  for (int i = 1; i < argc; i++) {
    if (i + 1 != argc) {
      // If we match the parameter
      if (strcmp(argv[i], "-r") == 0) {
        // set the frequency to the value of the next token interpreted as a
        // floating point number; if this ends up anything other than a
        // rate > 0 we'll have to quit
        talkerFrequency = atof(argv[i + 1]);
        // Skip the parameter value
        i++;
      }
    }
  }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Register the talker frequency adjustment service with the master
  ros::ServiceServer server = n.advertiseService("set_talker_frequency",
                                                 &setTalkerFrequency);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Declare our ROS rate object with a known good frequency
  ros::Rate loop_rate(1);

  // Make sure we can know when our frequency has been changed by a service call
  double previousTalkerFrequency;

  // Make sure we have a valid talker frequency
  if (talkerFrequency > 0) {
    loop_rate = ros::Rate(talkerFrequency);
    previousTalkerFrequency = talkerFrequency;
  } else {
    // Invalid rate specified; log fatal error and exit
    ROS_FATAL_STREAM("Talker can't start due to invalid frequency");
    return 1;
  }

  // Create a transformer broadcaster object so we can broadcast transforms
  tf::TransformBroadcaster br;

  // Create a transform object to hold the transform we will broadcast
  tf::Transform transform;

  // Set the origin of the child frame to be 0 units forward, 1 unit up, and 0.25 units
  // to the right of the parent frame origin
  transform.setOrigin(tf::Vector3(0.0, -0.25, 1.0));

  // Create a quaternion object for setting rotation information in the transform
  tf::Quaternion q;

  // Specify that the child frame is rotated 90 degrees clockwise around the
  // vertical (z) axis
  const double PI = std::atan(1.0) * 4;
  q.setRPY(0, 0, -PI / double(2.0));
  transform.setRotation(q);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok()) {
    // See if our frequency was adjusted
    if (talkerFrequency != previousTalkerFrequency) {
      // If the frequency is valid, adjust the message publishing rate
      if (talkerFrequency > 0) {
        loop_rate = ros::Rate(talkerFrequency);
        previousTalkerFrequency = talkerFrequency;
      } else {
        // Invalid rate specified; log fatal error and exit
        ROS_FATAL_STREAM("Talker can't continue due to invalid frequency");
        return 1;
      }
    }

    // Broadcast the transform using the transform broadcast object
    // The time that this transform is valid at is now (ros::Time::now())
    // Specify that this is a transform between "/world" (parent) and "/talk" (child)
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "/world", "/talk"));

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Johnny 5 is alive - need more input! " << count;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    // Turn over control temporarily
    ros::spinOnce();

    // Wait enough time to loop (and publish) at the requested rate
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
