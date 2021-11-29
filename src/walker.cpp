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
 * @file walker.cpp
 * @author Diane Ngo (dngo13)
 * @brief Source file for walker
 * @version 0.1
 * @date 2021-11-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/walker.hpp"

Walker::Walker() {
    vel_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1000);
    scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 500,
        &Walker::ObstacleCheck, this);
    obstacle_detected = false;
}

Walker::~Walker() {
    msg.linear.x = 0;
    msg.angular.z = 0;
    vel_pub.publish(msg);
}

void Walker::ObstacleCheck(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double min_dist = 0.15;
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] > min_dist) {
            min_dist = msg->ranges[i];
        }
    }
    front_dist = min_dist;
}

void Walker::RunWalker() {
    scan_sub = nh.subscribe<sensor_msgs::LaserScan> ("/scan", 1000,
        &Walker::ObstacleCheck, this);
    if (front_dist > 0.3) {
        ROS_INFO("Front is clear. ");
        msg.linear.x = 0.3;
        msg.angular.z = 0;
    } else {
        ROS_WARN("Obstacle detected, turning. ");
        obstacle_detected = true;
        msg.linear.x = 0;
        msg.angular.z = 0.3;
    }
    vel_pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    ros::NodeHandle nh;
    Walker walker;
    ros::Rate loop_rate(10.0);

    while (ros::ok()) {
        geometry_msgs::Twist msg;
        walker.RunWalker();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
