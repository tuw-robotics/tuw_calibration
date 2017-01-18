#ifndef LASER2CORNER_NODE_H
#define LASER2CORNER_NODE_H

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>

#include <tuw_geometry/tuw_geometry.h>

using namespace tuw;

class Laser2CornerNode
{
public:
  Laser2CornerNode(); // Constructor
private:
  ros::NodeHandle nh_;
  std::shared_ptr<tf::TransformListener> tf_listener_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
  MeasurementLaserPtr measurement_laser_;                  // laser measurements
  
  ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
  void callbackLaser(const sensor_msgs::LaserScan &_laser);
};

#endif //LASER2CORNER_NODE_H