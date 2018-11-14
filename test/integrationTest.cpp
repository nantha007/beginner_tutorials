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
 *  @file integrationTest.cpp
 *  @brief A program to test the talker Node
 *  @author Nantha Kumar Sunder
 *  @copyright 2018
 */
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "ros/ros.h"
/**
 *  @brief Test function to test talker node
 *  @return None
 */
TEST(TalkerTest, originValue) {
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform("/talk", "/world", ros::Time(0), ros::Duration(10));
  listener.lookupTransform("/talk", "/world", ros::Time(0), transform);
  EXPECT_EQ(-2, transform.getOrigin().y());
}
/**
 *  @brief main function to test the talker node
 *  @param argc is the number of argument
 *  @param argv is the arguments
 *  @return function that test the talker Node
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}

