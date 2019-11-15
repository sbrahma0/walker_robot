/**
 * BSD 3-Clause LICENSE
 *
 * Copyright (c) 2019, Sayan Brahma
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
 * @file walker.cpp
 *
 * Created on: Nov 14, 2019
 * @author: sayan Brahma
 * @brief ENPM808X ASSIHNMENT - Turtlebot
 */

// CPP header
#include <iostream>
// walker_robot class header
#include "walker.hpp"

walker::walker() {
  ROS_INFO_STREAM("turtlebot_walker node initialized");
  // Initialize class params
  collision = false;
  // advertise the publisher topic with rosmaster
  velPub = n.advertise < geometry_msgs::Twist
      > ("/cmd_vel_mux/input/navi", 1000);
  // SUbscribe to the laserscan topic
  sub = n.subscribe("/scan", 500, &walker::laser_ScanCallback, this);
  // define the initial velocities
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // publish the initial velocities
  velPub.publish(msg);
}

walker::~walker() {
  // stop the turtlebot before exiting
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // publish the  final velocities
  velPub.publish(msg);
}

void walker::laser_ScanCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Loop though the laserscan mesages to check collision
  for (int i = 0; i < msg->ranges.size(); ++i) {
    // Minimum threshold  distance for collision = 0.60
    if (msg->ranges[i] < 0.60) {
      collision = true;
      return;
    }
  }
  collision = false;
  return;
}

bool walker::detectObstacle() {
  // return the collision flag
  return collision;
}

void walker::navigate_Bot() {
  // Initialize the publisher freq
  ros::Rate loop_rate(10);
  // Implement till ros is running good
  while (ros::ok()) {
    // If obstacle is detected, turn the turtlebot
    if (detectObstacle()) {
      ROS_WARN_STREAM("Obstacle ahead, turning bot");
      // Stop the forward motion
      msg.linear.x = 0.0;
      // Rotate the bot
      msg.angular.z = -1.0;
    } else {
      ROS_INFO_STREAM("No obstacle ahead, moving straight");
      // If no obstacle, keep moving forward
      msg.linear.x = 0.3;
      msg.angular.z = 0.0;
    }
    // Publish the updated velocities
    velPub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}



