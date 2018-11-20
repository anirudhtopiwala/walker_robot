/**
 * BSD 3-Clause LICENSE
 *
 * Copyright (c) 2018, Anirudh Topiwala
 * All rights reserved. 
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
 *  @file    main.cpp
 *  @author  Anirudh Topiwala
 *  @copyright BSD License
 *
 *  @brief Implementing a walker node
 *
 *  @section DESCRIPTION
 *
 *  This program is an obstacle avoidance program. It achieves this 
 *  by moving forward unitl it meets an obstacle. When this happens
 *  it rotates in place until the obstacle is cleared and the again 
 *  move forwards.
 */

#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "walker.hpp"

int main(int argc, char **argv) {
  // initializing the node name as walker
  ros::init(argc, argv, "walker");

  walker Walk;

  // creating a node handle
  ros::NodeHandle n;

  // Subscribing to the topic laser scan data
  auto sensor = n.subscribe<sensor_msgs::
          LaserScan>("/scan", 50, &walker::laserScanCallback, &Walk);

  // Creating a publisher object which publishes to the given topic
  auto velocity = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);

  ros::Rate loop_rate(5);
  
  geometry_msgs::Twist msg;

  while (ros::ok()) {

  // Initiaizing msg with 0
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  if (Walk.inRange() == true) {
    // rotate until obstacle out is out of range
    msg.angular.z = 0.5;
    ROS_INFO_STREAM("Rotating to avoid Obstacle");
  } else {
    // moving forward
    msg.linear.x = 1;
    ROS_INFO_STREAM("Moving Forward and Exploring");
  }

  velocity.publish(msg);

  ros::spinOnce();

  loop_rate.sleep();
  }
  return 0;
}

