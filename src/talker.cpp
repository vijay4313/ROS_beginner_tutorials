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
 *  @brief	A simple ROS Publisher
 *
 *  @section DESCRIPTION
 *	Generates a simple ROS publisher
 *
 */

#include <sstream>
#include <cstdlib>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "beginner_tutorials/changeText.h"

// String to be published
extern std::string message = "My name is Venkat";


/*
 * @brief The routine that modifies
 * 	  the published text	  
 * @param req - The change request string
 * 	  res - change request response string
 */
bool modifyText(beginner_tutorials::changeText::Request  &req,
                beginner_tutorials::changeText::Response &res) {
    if (req.inpString.empty()) {
      ROS_ERROR_STREAM("No string specified");
    } else {
      message = req.inpString;
      ROS_WARN_STREAM("The Publisher Text changed");
    }
    res.outString = "The user changed the message to: " + req.inpString;
    return true;
    }

/*
 * @brief The main routine that generates
 * 		  a ROS simple publisher routine
 * @param argc - default ROS argument
 * 				 for command line remapping
 * 		  argv - default ROS argument
 * 		         for command line remapping
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("change_string", modifyText);
  tf::TransformBroadcaster br;  // frame transform broadcaster
  tf::Transform transform;  // frame transform handler
  // Publish talker message
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int rate = std::atoi(argv[1]);  // Set publishing rate
  if (rate <= 0) {
    ROS_FATAL_STREAM("The publisher rate has been set to 0 (or lesser)");
    rate = 10;
  } else {
    ROS_DEBUG_STREAM("The publisher rate changed to");
  }
  // %Tag(LOOP_RATE)%
    ros::Rate loop_rate(rate);
  // %EndTag(LOOP_RATE)%

    /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
    int count = 0;
    while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      std_msgs::String msg;
      msg.data = message.c_str();
      ROS_INFO_STREAM(msg.data.c_str());
      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      chatter_pub.publish(msg);
      // Publish Tf info
      transform.setOrigin(tf::Vector3(2.0, 3.0, 5.0));
      transform.setRotation(tf::Quaternion(0.2, 0.1, 0.1, 1));
      br.sendTransform(tf::StampedTransform(transform,
      ros::Time::now(), "world", "talker"));
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }


    return 0;
}
