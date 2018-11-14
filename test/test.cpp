/**
 * Copyright (c) 2018, Venkatraman Narayanan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *
 *  @file    talker.cpp
 *  @author  Venkatraman Narayanan (vijay4313)
 *  @copyright	MIT
 *  @date    11/13/2018
 *
 *  @brief	ROS package test routine
 *
 *  @section DESCRIPTION
 *	A simple testing routine for the ROS package
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "ros/service_client.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include <beginner_tutorials/changeText.h>


TEST(talkerNodeService, customMessageExistance) {
  ros::NodeHandle nh;  // Generate a ROS node handler
  auto client = nh.serviceClient<beginner_tutorials::changeText>("change_string");
  // Check if the service client is available
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

TEST(talkerNodeService, customMessageSuccess) {
  ros::NodeHandle nh;  // Generate a ROS node handler
  auto client = nh.serviceClient<beginner_tutorials::changeText>("change_string");
  beginner_tutorials::changeText msg;  // Create a placeholder for message
  msg.request.inpString = "ENPM808";  // Change the message
  client.call(msg.request, msg.response);
  EXPECT_STREQ("The user changed the message to: ENPM808", msg.response.outString.c_str());
}