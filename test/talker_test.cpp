/**
 * @copyright (c) 2017 Mark R. Jenkins.  All rights reserved.
 * @file talker_test.cpp
 *
 * @author MJenkins, ENPM 808X Spring 2017
 * @date Apr 10, 2017 - Creation
 *
 * @brief Tests for the "talker" node from ROS beginner_tutorials
 *
 * This is a rostest node designed to test the talker node developed originally from the ROS
 * beginner_tutorials, but expanded in functionality by my ENPM808X homework assignments.  At
 * the moment it only tests the setTalkerFrequency service.
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

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials2/TalkerFrequency.h"

// Set up the Talker setTalkerFrequency test using the GoogleTest macros
TEST (Talker, setTalkerFrequency) {
  // Create a node handle
  ros::NodeHandle nh;

  // Create a service client object for the "set_talker_frequency" service
  ros::ServiceClient client = nh
      .serviceClient<beginner_tutorials2::TalkerFrequency>(
      "set_talker_frequency");

  // Wait a bit for the service node to start up and offer the service
  bool serviceAvailable(client.waitForExistence(ros::Duration(1)));

  // The service should become available before too long
  EXPECT_TRUE(serviceAvailable);

  // Get srv object for the service calls
  beginner_tutorials2::TalkerFrequency srv;

  // Try to set a valid frequency of 1 message per second
  srv.request.msgsPerSecond = 5.5;
  bool success = client.call(srv);

  // The service call should return true to indicate it was successful
  EXPECT_TRUE(success);

  // The response frequency should match the requested frequency
  EXPECT_DOUBLE_EQ(double(5.5), srv.response.msgsPerSecond);
}


int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "talker_test");

  // Run all of the Google Test defined tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
