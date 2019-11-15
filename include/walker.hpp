/*
 * walker.hpp
 *
 *  Created on: 15-Nov-2019
 *      Author: sayan Brahma
 */

#ifndef WALKER_HPP_
#define WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

class walker {
 private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher velPub;
  bool collision;
  geometry_msgs::Twist msg;

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void navigateBot();

 public:
  /**
   *   @brief Constructor and destructor
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
   *   @brief function to move the bot around
   *
   *   @param  none
   *
   *   @return void
   */
  void navigate_Bot();
};

#endif /* WALKER_HPP_ */

