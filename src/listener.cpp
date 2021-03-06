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
 *  @file listener.cpp
 *  @brief A program to create a Node to subscribe messages on topic chatter
 *  @author Nantha Kumar Sunder
 *  @copyright 2018
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
/**
 * @brief Subscriber Node to topic chatter call back function
 * @param std_msg message received from subscription on topic chatter
 * @return none
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
/**
 *  @brief main function takes message on topic chatter
 *  @param argc is the number of argument
 *  @param argv is the arguments
 *  @return None
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  int bufferSize;
  int userBufferSize;
  ros::NodeHandle n;
  if (argc == 2) {
    userBufferSize = atoi(argv[1]);
  }
  ROS_INFO_STREAM("User entered BufferSize as :" << userBufferSize);
  if (argc != 2) {
    ROS_WARN_STREAM("There is No input argument!!");
    ROS_INFO_STREAM("Default value is taken");
    bufferSize = 1000;
  } else if (userBufferSize < 0) {
    ROS_FATAL_STREAM("BUFFER SIZE ENTERED IS NEGATIVE!!");
    ROS_INFO_STREAM("Taking Default value 1000 as Buffer Size.");
    bufferSize = 1000;
  } else if (userBufferSize == 0) {
    ROS_ERROR_STREAM("BUFFER SIZE ENTERED IS ZERO!!!");
    ROS_INFO_STREAM("Taking Default value 1000 as buffer size");
    bufferSize = 1000;
  } else {
    ROS_DEBUG_STREAM("The buffer size has been entered correctly!!");
    bufferSize = userBufferSize;
  }
  ros::Subscriber sub = n.subscribe("chatter", bufferSize, chatterCallback);
  ros::spin();
  return 0;
}

