#include <ros/ros.h>
#include <gtest/gtest.h>
#include "ros/service_client.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/String.h"
#include <beginner_tutorials/changeText.h>


TEST(talkerNodeService, customMessageExistance) {
  ros::NodeHandle nh;
  auto client = nh.serviceClient<beginner_tutorials::changeText>("change_string");
  EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}

TEST(talkerNodeService, customMessageSuccess) {
  ros::NodeHandle nh;
  auto client = nh.serviceClient<beginner_tutorials::changeText>("change_string");
  beginner_tutorials::changeText msg;
  msg.request.inpString = "ENPM808";
  client.call(msg.request, msg.response);
  EXPECT_STREQ("The user changed the message to: ENPM808", msg.response.outString.c_str());
}