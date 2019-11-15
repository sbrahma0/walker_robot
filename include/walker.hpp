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
/*
 * walker.hpp
 *
 *  Created on: 15-Nov-2019
 *      Author: sayan Brahma
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

/**
 *  @brief Class walker
 *
 *  The following class walker subscribes to laserScan data
 *  and  publishes command velocity to illustrate a walker mechanism by turtlebot
 */
class walker {
 private:
  // ROS Node handle object
  ros::NodeHandle n;
  // ROS subscriber object
  ros::Subscriber sub;
  // ROS publisher object
  ros::Publisher velPub;
  // Boolean flag to detect collision
  bool collision;
  // Message type to publish linear and angular velocities
  geometry_msgs::Twist msg;

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void navigateBot();

 public:
  /**
   *   @brief  Constructor for walker class
   *           Initializes the object
   *   @param  none
   *
   *   @return void
   */
  walker();
  ~walker();
  /**
   *   @brief  Callback function for subscriber to process laserScan data
   *
   *   @param  pointer to LaserScan mesage
   *
   *   @return void
   */
  void laser_ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   *   @brief  Function to detect obstable nearby
   *
   *   @param  none
   *
   *   @return true if object nearby, false otherwise
   */

  bool detectObstacle();

  /**
   *   @brief function to move the turtlebot around
   *
   *   @param  none
   *
   *   @return void
   */
  void navigate_Bot();
};

#endif  // INCLUDE_WALKER_HPP_

