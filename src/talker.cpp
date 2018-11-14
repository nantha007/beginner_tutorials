/**
 * MIT License
 * Copyright (c) 2018 Nantha Kumar Sunder
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/**
 *  @file talker.cpp
 *  @brief A program to create a Node to publish messages on topic chatter
 *  @author Nantha Kumar Sunder
 *  @copyright 2018
 */
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/messageModifier.h"
/**
 *  @brief main function takes message from server MessageModifier
 *         and send it to the channel chatter
 *  @param argc is the number of argument
 *  @param argv is the arguments
 *  @return None
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  int frequency = 10;
  std::string initString;
  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);
  ros::Rate loop_rate(frequency);
  ros::ServiceClient client = n.serviceClient
      < beginner_tutorials::messageModifier > ("MessageModiFier");
  beginner_tutorials::messageModifier str;
  str.request.newMessage = initString;
  client.call(str);
  initString = str.response.responseMessage;
  /*!<initializing a broadcaster*/
  tf::TransformBroadcaster br;
  tf::Transform transform;
  /**
   * The loop send the string "ROS tutorials" 
   * to the topic chatter along with the count 
   * of the loop iteration
   */
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << initString << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    /*!<Broadcasting the TF*/
    transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));
    transform.setRotation(tf::Quaternion(0.1, 0.2, 0.3, 0.4));
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

