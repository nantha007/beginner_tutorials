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
 *  @file messageModificationService.cpp
 *  @brief A program to change the message sent by the talker node.
 *  @author Nantha Kumar Sunder
 *  @copyright 2018
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/messageModifier.h"
/**
 * @brief service call back function
 * @param request is the Message request by client
 * @param response is the response message by server 
 * @return true
 */
bool messageModifiercallback(
    beginner_tutorials::messageModifier::Request &request,
    beginner_tutorials::messageModifier::Response &response) {
  response.responseMessage = "New Message  ";
  return true;
}
/**
 *  @brief main function creates a server for node talker
 *  @param argc is the number of argument
 *  @param argv is the arguments
 *  @return None
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "messageModificationService");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("MessageModiFier",
                                                  messageModifiercallback);
  ros::spin();
  return 0;
}

