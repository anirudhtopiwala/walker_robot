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
 *  @file    walker.cpp
 *  @author  Anirudh Topiwala
 *  @copyright BSD License
 *
 *  @brief Defining the functions for the class walker
 *
 *  @section DESCRIPTION
 *
 *  Defines the methods of class walker
 *
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "walker.hpp"
/**
 * Initializing Constructor for an object of walker class
 */
walker::walker() {
    // Initialing the range variable due to cppcheck warning
    range = false;
}
/**
 *Laserscan callback function to subscribe the topic
 */

void walker::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < 2) {
            range = true;
            ROS_DEBUG_STREAM("Range " << msg->ranges[i] << " less than 2m");
            return;
        }
    }

    range = false;
    ROS_DEBUG_STREAM("No obstacles in range");
    return;
}
/**
 * Returns the value of variable range
 */
bool walker::inRange() {
    return range;
}
/**
 * Calling Destructor for the object of walker class
 */
walker::~walker() {}