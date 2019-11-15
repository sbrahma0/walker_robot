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
  ros:Nodehandle n;
  ros::Subscriber sub;
  ros::Publisher velPub;
  bool collision;
  geometry_msgs::Twist msg;

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void navigateBot();

 public:
  turtlebotWalker();~turtlebotWalker();
}

#endif /* WALKER_HPP_ */

