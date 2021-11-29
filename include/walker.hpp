/**
 * Copyright (c) 2021, Diane Ngo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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


/**
 * @file walker.hpp
 * @author Diane Ngo (dngo13)
 * @brief Header file for walker
 * @version 0.1
 * @date 2021-11-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Walker {
 private:
    // Velocity Publisher
    ros::Publisher vel_pub;
    // Laser scan Subscriber
    ros::Subscriber scan_sub;
    // Twist message
    geometry_msgs::Twist msg;
    // Boolean if obstacle is detected
    bool obstacle_detected;
    // Front distance
    double front_dist;

 public:
    // Class Constructor
    Walker();
    // Class Destructor
    ~Walker();
    // Node handle
    ros::NodeHandle nh;
    /**
     * @brief Checks if there is obstacle in front of robot
     * 
     * @param msg 
     */
    void ObstacleCheck(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Runs the walker algorithm
     * 
     */
    void RunWalker();
};

#endif  // INCLUDE_WALKER_HPP_
